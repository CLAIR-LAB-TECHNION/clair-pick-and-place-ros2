#!/usr/bin/env bash
# Generate SVG files from Mermaid sources using @mermaid-js/mermaid-cli.
# Requires: Node.js and npx (install from https://nodejs.org/ or via apt: sudo apt install npm)
#
# Uses mermaid-cli@8.11.4 on Node < 18 (for Node 12 / older systems); otherwise latest.
# Usage: ./generate_svgs.sh
# Output: 01-hld-high-level-design.svg ... 07-class-diagram.svg in this directory.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if ! command -v npx &>/dev/null; then
  echo "Error: npx not found. Install Node.js (e.g. sudo apt install npm) and try again."
  exit 1
fi

# Node 18+ can use latest mermaid-cli; Node 12 needs older version (8.11.4)
NODE_VER=$(node -p "process.versions.node.split('.')[0]" 2>/dev/null || echo "0")
if [ "$NODE_VER" -lt 18 ] 2>/dev/null; then
  MMDC_PKG="@mermaid-js/mermaid-cli@8.11.4"
  echo "Using $MMDC_PKG (Node $(node -v) < 18)."
else
  MMDC_PKG="@mermaid-js/mermaid-cli"
fi

for mmd in 00-general-architecture.mmd 01-hld-high-level-design.mmd 02-control-flow-robot-pc.mmd 03-component-package.mmd 04-data-flow.mmd 05-deployment.mmd 06-execute-program-dispatch.mmd 07-class-diagram.mmd; do
  base="${mmd%.mmd}"
  echo "Generating $base.svg ..."
  npx -y "$MMDC_PKG" -i "$mmd" -o "$base.svg" -b transparent
done

echo "Done. SVGs written to $SCRIPT_DIR"
