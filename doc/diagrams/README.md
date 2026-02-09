# Architecture diagram sources and SVG export

This folder contains the **Mermaid sources** for all architecture diagrams and two ways to get **SVG** files.

## Contents

| File | Description |
|------|-------------|
| `01-hld-high-level-design.mmd` | HLD — User, Pick Up/Put Down/Traverse, Set Constraints/Configuration, Feasibility, IK, Error Handling, Grabriella |
| `02-control-flow-robot-pc.mmd` | Control flow: Robot (UR driver / Gazebo) + Control PC (ExecuteProgram, move/robmove/robpose, move_group) |
| `03-component-package.mmd` | Component & package diagram (User, Execution, Motion, Data, MoveIt, Sim, Real) |
| `04-data-flow.mmd` | Data flow: topics, actions, services (Publishers → Topics/Actions/Services → Consumers) |
| `05-deployment.mmd` | Deployment: Simulation vs Real robot + Common (ExecuteProgram / Pick) |
| `06-execute-program-dispatch.mmd` | ExecuteProgram step dispatch (YAML → Sequence → steps → RBT / Gripper / Constraints) |
| `07-class-diagram.mmd` | Class diagram (execution layer) |
| `render_diagrams.html` | Browser-based renderer with “Download SVG” for each diagram |
| `generate_svgs.sh` | Script to batch-generate SVGs using Node.js and `@mermaid-js/mermaid-cli` |

## How to get SVG files

### Option A: Browser (no Node required)

1. Open **`render_diagrams.html`** in a web browser (e.g. double-click or `file:///path/to/doc/diagrams/render_diagrams.html`).
2. Wait for all diagrams to render.
3. For each diagram, click **“Download 0X-….svg”** to save that SVG next to the HTML (or to your Downloads folder). Move them here if you want them in `doc/diagrams/`.

### Option B: Node.js (batch)

1. Install Node.js and npm if needed (e.g. `sudo apt install npm`).
2. In this directory, run:
   ```bash
   chmod +x generate_svgs.sh
   ./generate_svgs.sh
   ```
3. This produces `01-hld-high-level-design.svg` … `07-class-diagram.svg` in the same folder.

   **Note:** On Node.js 12 (e.g. default Ubuntu 22.04 `apt install npm`), the script automatically uses `@mermaid-js/mermaid-cli@8.11.4` to avoid engine requirements of the latest CLI. For Node 18+, the latest mermaid-cli is used. The **class diagram** (07) is written in 8.x‑compatible syntax (no `::` in names); for the full `ros2srrc::` / `rclcpp::` style, export that diagram from **render_diagrams.html** in a browser (Mermaid 11).

The diagrams in **`doc/ARCHITECTURE.md`** are the same as these sources; the SVGs are for embedding in docs, slides, or other tools.
