#!/usr/bin/env bash
# download_carla_agents.sh
# Downloads CARLA BehaviorAgent scripts from GitHub (tag 0.9.15) into src/agents/.
# Safe to run multiple times (idempotent).

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_ROOT="${WORKSPACE_ROOT}/src/agents"
BASE_URL="https://raw.githubusercontent.com/carla-simulator/carla/0.9.15/PythonAPI/carla/agents"

# ---------------------------------------------------------------------------
# Directory setup
# ---------------------------------------------------------------------------
echo "Creating directories..."
mkdir -p \
    "${DEST_ROOT}" \
    "${DEST_ROOT}/navigation" \
    "${DEST_ROOT}/tools"
echo "  OK: ${DEST_ROOT}"
echo "  OK: ${DEST_ROOT}/navigation"
echo "  OK: ${DEST_ROOT}/tools"

# ---------------------------------------------------------------------------
# File download list: "<remote_path>|<local_path>"
# ---------------------------------------------------------------------------
FILES=(
    "navigation/__init__.py|${DEST_ROOT}/navigation/__init__.py"
    "navigation/basic_agent.py|${DEST_ROOT}/navigation/basic_agent.py"
    "navigation/behavior_agent.py|${DEST_ROOT}/navigation/behavior_agent.py"
    "navigation/behavior_types.py|${DEST_ROOT}/navigation/behavior_types.py"
    "navigation/local_planner.py|${DEST_ROOT}/navigation/local_planner.py"
    "navigation/global_route_planner.py|${DEST_ROOT}/navigation/global_route_planner.py"
    "navigation/controller.py|${DEST_ROOT}/navigation/controller.py"
    "tools/__init__.py|${DEST_ROOT}/tools/__init__.py"
    "tools/misc.py|${DEST_ROOT}/tools/misc.py"
)

# ---------------------------------------------------------------------------
# Download each file
# ---------------------------------------------------------------------------
echo ""
echo "Downloading files from ${BASE_URL} ..."
FAILED=0

for entry in "${FILES[@]}"; do
    remote="${entry%%|*}"
    local_path="${entry##*|}"
    url="${BASE_URL}/${remote}"

    if curl -fsSL --retry 3 --retry-delay 2 -o "${local_path}" "${url}"; then
        echo "  OK: ${remote} -> ${local_path}"
    else
        echo "  FAIL: ${remote} (URL: ${url})" >&2
        FAILED=$((FAILED + 1))
    fi
done

# ---------------------------------------------------------------------------
# Create src/agents/__init__.py
# ---------------------------------------------------------------------------
echo ""
echo "Creating ${DEST_ROOT}/__init__.py ..."
cat > "${DEST_ROOT}/__init__.py" <<'PYEOF'
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
PYEOF
echo "  OK: ${DEST_ROOT}/__init__.py"

# ---------------------------------------------------------------------------
# Patch import paths: replace 'from agents.' with 'from src.agents.'
# ---------------------------------------------------------------------------
echo ""
echo "Patching import paths in downloaded files..."

while IFS= read -r -d '' f; do
    if sed -i 's/from agents\./from src.agents./g' "${f}"; then
        echo "  OK: patched ${f}"
    else
        echo "  FAIL: could not patch ${f}" >&2
        FAILED=$((FAILED + 1))
    fi
done < <(find "${DEST_ROOT}" -name "*.py" -print0)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
if [[ ${FAILED} -eq 0 ]]; then
    echo "All files downloaded and patched successfully."
else
    echo "Completed with ${FAILED} failure(s). Check messages above." >&2
    exit 1
fi
