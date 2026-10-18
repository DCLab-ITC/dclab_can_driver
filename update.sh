#!/usr/bin/env bash
# Updates this driver to the latest version on GitHub and rebuilds it.
#
#   ./update.sh            from GitHub (the robot needs internet)
#   ./update.sh FILE.zip   from a dclab_can_driver.zip copied to the robot
#                          (Super App Downloads, or GitHub's Download ZIP)
#
# A git clone is pulled (it stops if you changed files in it); a copy from a
# .zip is replaced (an old copy left in src/ would be a second can_driver).
# Then colcon builds the two packages in the workspace this folder is in
# (WORKSPACE/src/dclab_can_driver). Restart the node afterwards.
set -euo pipefail

REPO=https://github.com/DCLab-ITC/dclab_can_driver
here=$(cd "$(dirname "$0")" && pwd)
version() { sed -n 's/.*<version>\(.*\)<\/version>.*/\1/p' "$1/can_driver/package.xml" 2>/dev/null; }
before=$(version "$here")

if [ $# -eq 0 ] && [ -d "$here/.git" ]; then
  if [ -n "$(git -C "$here" status --porcelain --untracked-files=no)" ]; then
    echo "You changed files in $here (git status): commit or undo them, then run this again." >&2
    exit 1
  fi
  git -C "$here" pull --ff-only "$REPO" main
else
  work=$(mktemp -d)
  trap 'rm -rf "$work"' EXIT
  if [ $# -gt 0 ]; then
    cp "$1" "$work/new.zip"
  else
    echo "Downloading $REPO ..."
    python3 - "$REPO/archive/refs/heads/main.zip" "$work/new.zip" <<'EOF'
import sys, urllib.request
with urllib.request.urlopen(sys.argv[1], timeout=60) as r, open(sys.argv[2], "wb") as f:
    f.write(r.read())
EOF
  fi
  python3 - "$work/new.zip" "$work/new" <<'EOF'
import sys, zipfile
zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])
EOF
  # one top folder: dclab_can_driver/ (Super App) or dclab_can_driver-main/ (GitHub)
  new=$(find "$work/new" -mindepth 1 -maxdepth 1 -type d | head -n 1)
  if [ ! -f "$new/can_driver/package.xml" ] || [ ! -d "$new/custom_messages" ]; then
    echo "That is not a dclab_can_driver archive." >&2
    exit 1
  fi
  chmod +x "$new/update.sh" 2>/dev/null || true
  mv "$here" "$work/old"
  mv "$new" "$here"
fi

after=$(version "$here")
echo "can_driver ${before:-?} -> ${after:-?}"

src=$(dirname "$here")
if [ "$(basename "$src")" != src ]; then
  echo "Not in a ROS 2 workspace (WORKSPACE/src/dclab_can_driver): build it with colcon yourself."
  exit 0
fi
if ! command -v colcon >/dev/null; then
  distro=${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | head -n 1)}
  if [ -n "$distro" ] && [ -f "/opt/ros/$distro/setup.bash" ]; then
    set +u; source "/opt/ros/$distro/setup.bash"; set -u
  fi
fi
cd "$(dirname "$src")"
colcon build --packages-select custom_messages can_driver
echo "Done. Run 'source install/setup.bash' in open terminals and restart the node."
