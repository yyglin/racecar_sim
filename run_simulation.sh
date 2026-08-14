#!/usr/bin/env bash

set -Eeuo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${repo_dir}"

if ! command -v docker >/dev/null 2>&1; then
  echo "错误：宿主机找不到 docker 命令。" >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "错误：当前 Docker 没有可用的 Compose 插件。" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "错误：宿主机 DISPLAY 为空，请在图形桌面的终端中运行本脚本。" >&2
  exit 1
fi

"${repo_dir}/allow_x11.sh"

export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g)}"
export DISPLAY

docker compose up -d simulation

docker compose exec simulation bash -lc '
  set -e

  source /opt/ros/jazzy/setup.bash
  echo "已加载 /opt/ros/jazzy/setup.bash"

  if [[ -f /Fast_Segmentation_ws/install/setup.bash ]]; then
    source /Fast_Segmentation_ws/install/setup.bash
    echo "已加载 /Fast_Segmentation_ws/install/setup.bash"
  else
    echo "提示：/Fast_Segmentation_ws/install/setup.bash 不存在，跳过算法工作空间。"
  fi

  if [[ -f /simulation_ws/install/setup.bash ]]; then
    source /simulation_ws/install/setup.bash
    echo "已加载 /simulation_ws/install/setup.bash"
  else
    echo "提示：/simulation_ws/install/setup.bash 不存在；请先执行 colcon build --symlink-install。"
  fi

  cd /simulation_ws
  echo "ROS_DISTRO=${ROS_DISTRO:-unknown}，ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  echo "现在可以运行：ros2 launch racecar_gazebo simulation.launch.py gui:=true"
  exec bash --noprofile --norc -i
'
