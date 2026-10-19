#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

if [[ -f "${PWD}/SOURCE_MANIFEST.json" && "$(basename "$(dirname "${PWD}")")" == "releases" ]]; then
  project_root="$(cd "${PWD}/../.." && pwd)"
else
  project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fi
prefix="${project_root}/envs/g3b-humble-overlay"
cache="${project_root}/var/cache/g3b-humble-overlay-debs"
mkdir -p "$prefix" "$cache"

packages=(
  ros-humble-slam-toolbox
  ros-humble-nav2-bringup
  ros-humble-nav2-rviz-plugins
  xvfb
  xdotool
)

apt-get -s install "${packages[@]}" |
  awk '/^Inst / {print $2}' |
  sort -u > "$cache/package-list.txt"

if [[ ! -s "$cache/package-list.txt" ]]; then
  echo "No overlay packages resolved" >&2
  exit 1
fi

(
  cd "$cache"
  xargs -a package-list.txt -n 8 apt-get download
  rm -f ./*.deb.sha256
  sha256sum ./*.deb > overlay-debs.sha256
)

for package in "${packages[@]}"; do
  apt-cache show "$package" |
    awk -v name="$package" '
      /^Package:/ {pkg=$2}
      /^Version:/ && pkg==name {print pkg "=" $2; exit}
    '
done > "$cache/top-level-lock.txt"

find "$prefix" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
while IFS= read -r deb; do
  dpkg-deb -x "$cache/$deb" "$prefix"
done < <(sed 's/^[0-9a-f]*  //' "$cache/overlay-debs.sha256")

python3 - "$prefix" "$cache" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

prefix = Path(sys.argv[1])
cache = Path(sys.argv[2])
packages = []
for line in (cache / "overlay-debs.sha256").read_text(encoding="utf-8").splitlines():
    digest, name = line.split("  ", 1)
    packages.append({"deb_sha256": digest, "deb_name": name})
payload = {
    "schema_version": 1,
    "installation": "server_local_overlay",
    "base_ros": "/opt/ros/humble",
    "source": "apt-get download and dpkg-deb -x without system installation",
    "top_level_lock": (cache / "top-level-lock.txt").read_text(encoding="utf-8").splitlines(),
    "packages": packages,
}
(prefix / "overlay_manifest.json").write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

cat > "$prefix/overlay_env.sh" <<EOF
source /opt/ros/humble/setup.bash
export G3B_ROS_OVERLAY_PREFIX="$prefix"
export AMENT_PREFIX_PATH="$prefix/opt/ros/humble\${AMENT_PREFIX_PATH:+:\$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$prefix/opt/ros/humble\${CMAKE_PREFIX_PATH:+:\$CMAKE_PREFIX_PATH}"
export COLCON_PREFIX_PATH="$prefix/opt/ros/humble\${COLCON_PREFIX_PATH:+:\$COLCON_PREFIX_PATH}"
export LD_LIBRARY_PATH="$prefix/usr/lib/x86_64-linux-gnu:$prefix/usr/lib:$prefix/opt/ros/humble/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export PATH="$prefix/usr/bin:$prefix/opt/ros/humble/bin:\$PATH"
export PKG_CONFIG_PATH="$prefix/usr/lib/x86_64-linux-gnu/pkgconfig:$prefix/usr/share/pkgconfig\${PKG_CONFIG_PATH:+:\$PKG_CONFIG_PATH}"
export PYTHONPATH="$prefix/opt/ros/humble/local/lib/python3.10/dist-packages:$prefix/opt/ros/humble/lib/python3.10/site-packages\${PYTHONPATH:+:\$PYTHONPATH}"
EOF

echo "G3B ROS overlay: $prefix"
