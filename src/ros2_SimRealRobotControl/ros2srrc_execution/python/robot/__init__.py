# Package for robot clients
# Import RBT class from robot.py file in the same directory
import importlib.util
import os

# Get the path to robot.py
_robot_py_path = os.path.join(os.path.dirname(__file__), 'robot.py')

# Load robot.py as a module with a unique name to avoid conflicts
_spec = importlib.util.spec_from_file_location("_robot_py_module", _robot_py_path)
_robot_py = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_robot_py)

# Export RBT class
RBT = _robot_py.RBT

__all__ = ['RBT']
