#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
# The base CUDA image contains placeholder Tegra libraries. This desktop uses Mesa.
unset LD_LIBRARY_PATH
source "$ROOT/posnav/deploy/env.sh"
source "$ROOT/.cache/fastlio_ws/install/setup.bash"
source "$ROOT/.cache/rviz_panel_ws/install/setup.bash"
export QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
exec /usr/bin/python3 "$ROOT/tools/g1_live_desktop.py" "$@"
