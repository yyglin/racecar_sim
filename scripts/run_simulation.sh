#!/usr/bin/env bash

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
compose_file="${project_root}/compose.yaml"
container_root="/Fast_Segmentation_of_3D_Point_Clouds_for_Ground_Vehicles"

if ! command -v docker >/dev/null 2>&1; then
  echo "错误：未找到 docker 命令。" >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "错误：未找到 Docker Compose V2（docker compose）。" >&2
  exit 1
fi

if [[ ! -f "${compose_file}" ]]; then
  echo "错误：找不到 Compose 文件：${compose_file}" >&2
  exit 1
fi

if [[ ! -x "${script_dir}/allow_x11.sh" ]]; then
  echo "错误：找不到可执行的 ${script_dir}/allow_x11.sh" >&2
  exit 1
fi

export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g)}"

x11_allowed=false
cleanup() {
  if [[ "${x11_allowed}" == true ]]; then
    "${script_dir}/allow_x11.sh" revoke >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "[1/4] 配置 X11 显示权限……"
"${script_dir}/allow_x11.sh" allow
x11_allowed=true

echo "[2/4] 启动 algorithm 和 simulation 容器……"
docker compose --project-directory "${project_root}" -f "${compose_file}" \
  up -d algorithm simulation

echo "[3/4] 增量编译地面分割算法和仿真包……"
docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T algorithm bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd '${container_root}/Fast_Segmentation_ws' && \
   colcon build --symlink-install --packages-select fast_ground_segmenter"

docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T simulation bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source '${container_root}/Fast_Segmentation_ws/install/setup.bash' && \
   cd '${container_root}/simulation_ws' && \
   colcon build --symlink-install --packages-select racecar_description racecar_gazebo"

if docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T simulation pgrep -f \
  "ros2 launch racecar_gazebo simulation.launch.py" >/dev/null 2>&1; then
  echo "错误：simulation.launch.py 已经在 simulation 容器中运行。" >&2
  echo "请先在原来的终端中按 Ctrl+C 停止它，再重新运行本脚本。" >&2
  exit 1
fi

echo "[4/4] 启动 Gazebo、RViz 和地面分割算法……"
echo "按 Ctrl+C 可停止本次 launch。"
docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T simulation bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source '${container_root}/Fast_Segmentation_ws/install/setup.bash' && \
   source '${container_root}/simulation_ws/install/setup.bash' && \
   ros2 launch racecar_gazebo simulation.launch.py"
