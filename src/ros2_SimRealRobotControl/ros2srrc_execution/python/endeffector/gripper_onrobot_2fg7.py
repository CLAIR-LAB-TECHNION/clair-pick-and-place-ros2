# gripper_onrobot_2fg7.py
# OnRobot 2FG7 backend with two protocols:
#
# 1) XML-RPC over HTTP (default, recommended for Oren-course / real lab):
#    http://<robot_ip>:41414 — methods twofg_grip_external, twofg_get_* etc.
#    No URScript; control via OnRobot 2FG7 XML-RPC server on the robot.
#
# 2) URScript over TCP (optional): port 30002, send rq_set_width(...) etc.
#    Set protocol:=urscript and configure open_script_template, close_script_template.
#
# Params: robot_ip, protocol (xmlrpc | urscript), gripper_id, force, speed,
# open_width_mm, (URScript-only: open_script_template, close_script_template, ...).

import logging
import socket
import time
from typing import Any, Tuple
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

import xmlrpc.client

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from .gripper_interface import GripperInterface

_LOG = logging.getLogger(__name__)

# Protocol names
PROTOCOL_XMLRPC = "xmlrpc"
PROTOCOL_URSCRIPT = "urscript"
_DEFAULT_PROTOCOL = PROTOCOL_XMLRPC

# XML-RPC (Oren-course / real lab)
_DEFAULT_XMLRPC_PORT = 41414
_DEFAULT_GRIPPER_ID = 0
_DEFAULT_FORCE = 50
_DEFAULT_SPEED = 50
# Force used for "open/release" (move to max width)
_RELEASE_FORCE = 80
# Force/speed limits from reference (2FG7 electrical system)
_FORCE_MIN = 20

# URScript fallback
_DEFAULT_OPEN_SCRIPT = "rq_set_width(110)\n"
_DEFAULT_CLOSE_SCRIPT = "rq_set_width({width})\n"
_DEFAULT_JAW_WIDTH_OPEN_MM = 110.0
_DEFAULT_URSCRIPT_PORT = 30002
_DEFAULT_WAIT_AFTER_CMD = 0.75
_DEFAULT_TCP_TIMEOUT = 5.0

_BRINGUP_PARAMS_NODE_NAME = "onrobot_2fg7_bringup_params"
_ROBOT_CLIENT_NODE_NAME = "ros2srrc_RobMove_Client"
_ROBOT_IP_ERROR_MSG = (
    "OnRobot 2FG7: robot_ip not set. Set it with:\n"
    "  --ros-args -p OnRobot2FG7_param_reader.robot_ip:=<ROBOT_IP>"
)


def _get_robot_ip_from_node(param_reader_node: Node, robot_node_name: str) -> str:
    try:
        client = param_reader_node.create_client(
            GetParameters,
            f"/{robot_node_name}/get_parameters",
        )
        if not client.wait_for_service(timeout_sec=1.0):
            return ""
        req = GetParameters.Request()
        req.names = ["robot_ip"]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(param_reader_node, future, timeout_sec=2.0)
        if not future.done() or not future.result():
            return ""
        result = future.result()
        if not result.values:
            return ""
        pv = result.values[0]
        if pv.type == 4:
            return pv.string_value or ""
        return ""
    except Exception:
        return ""


# ---------- XML-RPC client (stdlib only: urllib + xmlrpc.client.loads) ----------


def _build_xml_request(method_name: str, params: list) -> str:
    """Build XML-RPC methodCall body."""
    param_elts = []
    for p in params:
        if isinstance(p, int):
            param_elts.append(f"<param><value><int>{p}</int></value></param>")
        elif isinstance(p, float):
            param_elts.append(f"<param><value><double>{p}</double></value></param>")
        elif isinstance(p, bool):
            param_elts.append(f"<param><value><boolean>{1 if p else 0}</boolean></value></param>")
        else:
            param_elts.append(f"<param><value><string>{p}</string></value></param>")
    params_str = "\n".join(param_elts)
    return (
        '<?xml version="1.0"?>\n'
        "<methodCall>\n"
        f"  <methodName>{method_name}</methodName>\n"
        "  <params>\n"
        f"    {params_str}\n"
        "  </params>\n"
        "</methodCall>"
    )


def _xmlrpc_call(robot_ip: str, port: int, method_name: str, params: list, timeout: float) -> Tuple[bool, str, Any]:
    """POST XML-RPC request to http://robot_ip:port. Returns (success, message, result). Uses stdlib only."""
    if not (robot_ip or "").strip():
        return False, _ROBOT_IP_ERROR_MSG, None
    url = f"http://{robot_ip.strip()}:{port}"
    body = _build_xml_request(method_name, params).replace("\r\n", "").encode("utf-8")
    req = Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/x-www-form-urlencoded")
    start = time.monotonic()
    try:
        with urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8")
        result, _ = xmlrpc.client.loads(raw)
        exec_time = time.monotonic() - start
        return True, "OK", result[0] if result else None
    except (URLError, HTTPError, OSError) as e:
        return False, f"OnRobot 2FG7 XML-RPC: {type(e).__name__}: {e}", None
    except Exception as e:
        return False, f"OnRobot 2FG7 XML-RPC: {type(e).__name__}: {e}", None


class _TwoFG7XmlRpcClient:
    """Minimal OnRobot 2FG7 XML-RPC client (twofg_* methods over HTTP :41414). Stdlib only."""

    def __init__(self, robot_ip: str, port: int, gripper_id: int, timeout: float = 5.0):
        self.robot_ip = (robot_ip or "").strip()
        self.port = port
        self.gid = gripper_id
        self.timeout = timeout
        self._max_ext = None
        self._min_ext = None
        self._max_force = None

    def _call(self, method: str, *params) -> Tuple[bool, str, Any]:
        return _xmlrpc_call(
            self.robot_ip, self.port, method, [self.gid] + list(params), self.timeout
        )

    def get_max_external_width(self) -> Tuple[bool, str, float]:
        ok, msg, val = self._call("twofg_get_max_external_width")
        if ok and val is not None:
            self._max_ext = float(val)
        return ok, msg, float(val) if val is not None else 0.0

    def get_min_external_width(self) -> Tuple[bool, str, float]:
        ok, msg, val = self._call("twofg_get_min_external_width")
        if ok and val is not None:
            self._min_ext = float(val)
        return ok, msg, float(val) if val is not None else 0.0

    def get_max_force(self) -> Tuple[bool, str, int]:
        ok, msg, val = self._call("twofg_get_max_force")
        if ok and val is not None:
            self._max_force = int(val)
        return ok, msg, int(val) if val is not None else 0

    def grip_external(self, target_width_mm: float, target_force: int, speed: int) -> Tuple[bool, str, int]:
        """twofg_grip_external(id, target_width, target_force, speed). Returns (ok, msg, status)."""
        return self._call("twofg_grip_external", target_width_mm, target_force, speed)

    def refresh_limits(self) -> Tuple[bool, str]:
        """Query gripper for max/min width and max force. Returns (ok, message)."""
        ok, msg, _ = self.get_max_external_width()
        if not ok:
            return False, msg
        ok, msg, _ = self.get_min_external_width()
        if not ok:
            return False, msg
        ok, msg, _ = self.get_max_force()
        return ok, msg


# ---------- URScript helpers (unchanged) ----------


def _send_urscript(host: str, port: int, script: str, timeout: float, wait_after: float) -> Tuple[bool, str, float]:
    if not host or not host.strip():
        return False, _ROBOT_IP_ERROR_MSG, 0.0
    script = script.strip()
    if not script.endswith("\n"):
        script += "\n"
    start = time.monotonic()
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host.strip(), port))
        sock.sendall(script.encode("utf-8"))
        if wait_after > 0:
            time.sleep(wait_after)
        sock.close()
        return True, "OK", time.monotonic() - start
    except socket.timeout:
        return False, f"OnRobot 2FG7: TCP timeout ({timeout}s) connecting to {host}:{port}", time.monotonic() - start
    except Exception as e:
        return False, f"OnRobot 2FG7: {type(e).__name__}: {e}", time.monotonic() - start


def _format_template(template: str, width_mm: float, jaw_width_open_mm: float, force: float, speed: float) -> str:
    return template.format(
        width=int(round(width_mm)),
        width_mm=width_mm,
        jaw_width_open_mm=jaw_width_open_mm,
        force=int(round(force)),
        speed=int(round(speed)),
    )


# ---------- Param reader ----------


class _ParamReader(Node):
    def __init__(self):
        super().__init__("OnRobot2FG7_param_reader")
        self.declare_parameter("robot_ip", "")
        self.declare_parameter("protocol", _DEFAULT_PROTOCOL)
        self.declare_parameter("gripper_id", _DEFAULT_GRIPPER_ID)
        self.declare_parameter("xmlrpc_port", _DEFAULT_XMLRPC_PORT)
        self.declare_parameter("force", _DEFAULT_FORCE)
        self.declare_parameter("speed", _DEFAULT_SPEED)
        self.declare_parameter("open_width_mm", 0.0)  # 0 = use max from gripper (xmlrpc)
        self.declare_parameter("urscript_port", _DEFAULT_URSCRIPT_PORT)
        self.declare_parameter("wait_after_cmd", _DEFAULT_WAIT_AFTER_CMD)
        self.declare_parameter("open_script_template", _DEFAULT_OPEN_SCRIPT)
        self.declare_parameter("close_script_template", _DEFAULT_CLOSE_SCRIPT)
        self.declare_parameter("jaw_width_open_mm", _DEFAULT_JAW_WIDTH_OPEN_MM)
        self.declare_parameter("tcp_timeout", _DEFAULT_TCP_TIMEOUT)

    def get_robot_ip(self) -> str:
        ip = self.get_parameter("robot_ip").get_parameter_value().string_value or ""
        ip = (ip or "").strip()
        if ip:
            return ip
        ip = _get_robot_ip_from_node(self, _BRINGUP_PARAMS_NODE_NAME)
        ip = (ip or "").strip()
        if ip:
            return ip
        ip = _get_robot_ip_from_node(self, _ROBOT_CLIENT_NODE_NAME)
        ip = (ip or "").strip()
        if ip:
            return ip
        raise RuntimeError(_ROBOT_IP_ERROR_MSG)

    def get_config(self):
        try:
            robot_ip = self.get_robot_ip()
        except RuntimeError:
            robot_ip = ""
        return {
            "robot_ip": robot_ip,
            "protocol": (self.get_parameter("protocol").get_parameter_value().string_value or _DEFAULT_PROTOCOL).strip().lower(),
            "gripper_id": self.get_parameter("gripper_id").value,
            "xmlrpc_port": self.get_parameter("xmlrpc_port").value,
            "force": self.get_parameter("force").value,
            "speed": self.get_parameter("speed").value,
            "open_width_mm": self.get_parameter("open_width_mm").value,
            "urscript_port": self.get_parameter("urscript_port").value,
            "wait_after_cmd": self.get_parameter("wait_after_cmd").value,
            "open_script_template": self.get_parameter("open_script_template").get_parameter_value().string_value,
            "close_script_template": self.get_parameter("close_script_template").get_parameter_value().string_value,
            "jaw_width_open_mm": self.get_parameter("jaw_width_open_mm").value,
            "tcp_timeout": self.get_parameter("tcp_timeout").value,
        }


# ---------- Backend ----------


class OnRobot2FG7Backend(GripperInterface):
    """
    OnRobot 2FG7: protocol "xmlrpc" (default) uses HTTP :41414 (twofg_grip_external, twofg_get_*).
    protocol "urscript" uses TCP :30002 and script templates.
    """

    def __init__(
        self,
        robot_ip: str | None = None,
        protocol: str | None = None,
        gripper_id: int | None = None,
        xmlrpc_port: int | None = None,
        force: int | float | None = None,
        speed: int | float | None = None,
        open_width_mm: float | None = None,
        urscript_port: int | None = None,
        wait_after_cmd: float | None = None,
        open_script_template: str | None = None,
        close_script_template: str | None = None,
        jaw_width_open_mm: float | None = None,
        tcp_timeout: float | None = None,
    ):
        rclpy.init() if not rclpy.ok() else None
        self._param_reader = _ParamReader()
        self._init_overrides = {
            "robot_ip": robot_ip,
            "protocol": protocol,
            "gripper_id": gripper_id,
            "xmlrpc_port": xmlrpc_port,
            "force": force,
            "speed": speed,
            "open_width_mm": open_width_mm,
            "urscript_port": urscript_port,
            "wait_after_cmd": wait_after_cmd,
            "open_script_template": open_script_template,
            "close_script_template": close_script_template,
            "jaw_width_open_mm": jaw_width_open_mm,
            "tcp_timeout": tcp_timeout,
        }
        self._xmlrpc_client: _TwoFG7XmlRpcClient | None = None

    def _get_config(self):
        cfg = self._param_reader.get_config()
        for k, v in self._init_overrides.items():
            if v is not None:
                cfg[k] = v
        return cfg

    def _get_xmlrpc_client(self):
        cfg = self._get_config()
        if not (cfg["robot_ip"] or "").strip():
            return None, cfg, _ROBOT_IP_ERROR_MSG
        if self._xmlrpc_client is None:
            self._xmlrpc_client = _TwoFG7XmlRpcClient(
                cfg["robot_ip"],
                int(cfg["xmlrpc_port"]),
                int(cfg["gripper_id"]),
                float(cfg["tcp_timeout"]),
            )
        return self._xmlrpc_client, cfg, None

    def open(self):
        cfg = self._get_config()
        if not (cfg["robot_ip"] or "").strip():
            return {"Success": False, "Message": _ROBOT_IP_ERROR_MSG, "ExecTime": 0.0}

        protocol = (cfg.get("protocol") or _DEFAULT_PROTOCOL).strip().lower()

        if protocol == PROTOCOL_XMLRPC:
            return self._open_xmlrpc(cfg)
        return self._open_urscript(cfg)

    def _open_xmlrpc(self, cfg: dict):
        client, cfg, err = self._get_xmlrpc_client()
        if err:
            return {"Success": False, "Message": err, "ExecTime": 0.0}
        ok, msg, _ = client.refresh_limits()
        if not ok:
            return {"Success": False, "Message": msg, "ExecTime": 0.0}
        open_mm = float(cfg.get("open_width_mm") or 0)
        if open_mm <= 0 and client._max_ext is not None:
            open_mm = client._max_ext
        if open_mm <= 0:
            open_mm = 110.0
        max_force = client._max_force if client._max_force is not None else 100
        force = max(_FORCE_MIN, min(int(cfg.get("force") or _DEFAULT_FORCE), max_force))
        speed = max(1, min(100, int(cfg.get("speed") or _DEFAULT_SPEED)))
        if client._min_ext is not None and open_mm < client._min_ext:
            open_mm = client._min_ext
        if client._max_ext is not None and open_mm > client._max_ext:
            open_mm = client._max_ext
        # Open = move to open width with release force (reference uses 80)
        ok, msg, status = client.grip_external(open_mm, _RELEASE_FORCE, speed)
        return {"Success": ok and (status == 0), "Message": msg, "ExecTime": 0.0}

    def _open_urscript(self, cfg: dict):
        template = (cfg.get("open_script_template") or _DEFAULT_OPEN_SCRIPT).strip()
        jaw = float(cfg.get("jaw_width_open_mm") or _DEFAULT_JAW_WIDTH_OPEN_MM)
        force = float(cfg.get("force") or _DEFAULT_FORCE)
        speed = float(cfg.get("speed") or _DEFAULT_SPEED)
        script = _format_template(template, jaw, jaw, force, speed)
        if not script.endswith("\n"):
            script += "\n"
        success, msg, exec_time = _send_urscript(
            cfg["robot_ip"], int(cfg["urscript_port"]), script,
            float(cfg["tcp_timeout"]), float(cfg["wait_after_cmd"]),
        )
        return {"Success": success, "Message": msg, "ExecTime": exec_time}

    def close(self, percent: float = 1.0):
        p = self._clamp_percent(percent)
        cfg = self._get_config()
        if not (cfg["robot_ip"] or "").strip():
            return {"Success": False, "Message": _ROBOT_IP_ERROR_MSG, "ExecTime": 0.0}

        protocol = (cfg.get("protocol") or _DEFAULT_PROTOCOL).strip().lower()

        if protocol == PROTOCOL_XMLRPC:
            return self._close_xmlrpc(cfg, p)
        return self._close_urscript(cfg, p)

    def _close_xmlrpc(self, cfg: dict, percent: float):
        client, cfg, err = self._get_xmlrpc_client()
        if err:
            return {"Success": False, "Message": err, "ExecTime": 0.0}
        ok, msg, _ = client.refresh_limits()
        if not ok:
            return {"Success": False, "Message": msg, "ExecTime": 0.0}
        max_ext = client._max_ext if client._max_ext is not None else 110.0
        min_ext = client._min_ext if client._min_ext is not None else 0.0
        # percent 1 = closed (min), percent 0 = open (max)
        width_mm = min_ext + (1.0 - percent) * (max_ext - min_ext)
        width_mm = max(min_ext, min(max_ext, width_mm))
        force = max(_FORCE_MIN, min(int(cfg.get("force") or _DEFAULT_FORCE), client._max_force or 100))
        speed = max(1, min(100, int(cfg.get("speed") or _DEFAULT_SPEED)))
        ok, msg, status = client.grip_external(width_mm, force, speed)
        return {"Success": ok and (status == 0), "Message": msg, "ExecTime": 0.0}

    def _close_urscript(self, cfg: dict, percent: float):
        jaw_open = float(cfg.get("jaw_width_open_mm") or _DEFAULT_JAW_WIDTH_OPEN_MM)
        width_mm = jaw_open * (1.0 - percent)
        force = float(cfg.get("force") or _DEFAULT_FORCE)
        speed = float(cfg.get("speed") or _DEFAULT_SPEED)
        template = cfg.get("close_script_template") or _DEFAULT_CLOSE_SCRIPT
        script = _format_template(template, width_mm, jaw_open, force, speed)
        if not script.endswith("\n"):
            script += "\n"
        success, msg, exec_time = _send_urscript(
            cfg["robot_ip"], int(cfg["urscript_port"]), script,
            float(cfg["tcp_timeout"]), float(cfg["wait_after_cmd"]),
        )
        return {"Success": success, "Message": msg, "ExecTime": exec_time}

    def query_limits_xmlrpc(self):
        """For self-test: return (ok, message, dict with max_ext, min_ext, max_force) or (False, msg, None)."""
        client, cfg, err = self._get_xmlrpc_client()
        if err:
            return False, err, None
        ok, msg, max_ext = client.get_max_external_width()
        if not ok:
            return False, msg, None
        ok, msg, min_ext = client.get_min_external_width()
        if not ok:
            return False, msg, None
        ok, msg, max_force = client.get_max_force()
        if not ok:
            return False, msg, None
        return True, "OK", {"max_ext": max_ext, "min_ext": min_ext, "max_force": max_force}
