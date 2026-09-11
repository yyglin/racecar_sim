#!/usr/bin/env bash

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
compose_file="${project_root}/compose.yaml"
container_root="/Fast_Segmentation_of_3D_Point_Clouds_for_Ground_Vehicles"

network_interface="eno1"
host_ip="192.168.1.50"
lidar_ip="192.168.1.133"
foxglove_address="127.0.0.1"
foxglove_port="8765"
build_image=false

usage() {
  cat <<'EOF'
用法：./scripts/run_mid360.sh [选项]

配置电脑网卡并启动 Mid-360、地面分割算法和 Foxglove Bridge。
默认在本机 Foxglove 中连接：ws://localhost:8765

选项：
  --interface NAME          雷达连接的有线网卡（默认：eno1）
  --host-ip ADDRESS         电脑雷达网卡 IPv4 地址（默认：192.168.1.50）
  --lidar-ip ADDRESS        Mid-360 IPv4 地址（默认：192.168.1.133）
  --foxglove-address ADDR   Bridge 监听地址（默认：127.0.0.1）
  --foxglove-port PORT      Bridge 监听端口（默认：8765）
  --build-image             启动前重新构建 algorithm Docker 镜像
  -h, --help                显示帮助

示例：
  ./scripts/run_mid360.sh
  ./scripts/run_mid360.sh --lidar-ip 192.168.1.112
  ./scripts/run_mid360.sh --foxglove-address 0.0.0.0

按 Ctrl+C 可停止雷达驱动、算法和 Foxglove Bridge。
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "未找到命令：$1"
}

run_as_root() {
  if ((EUID == 0)); then
    "$@"
    return
  fi

  command -v sudo >/dev/null 2>&1 || die "需要管理员权限，但未找到 sudo"
  sudo "$@"
}

while (($# > 0)); do
  case "$1" in
    --interface)
      (($# >= 2)) || die "--interface 缺少参数"
      network_interface="$2"
      shift 2
      ;;
    --host-ip)
      (($# >= 2)) || die "--host-ip 缺少参数"
      host_ip="$2"
      shift 2
      ;;
    --lidar-ip)
      (($# >= 2)) || die "--lidar-ip 缺少参数"
      lidar_ip="$2"
      shift 2
      ;;
    --foxglove-address)
      (($# >= 2)) || die "--foxglove-address 缺少参数"
      foxglove_address="$2"
      shift 2
      ;;
    --foxglove-port)
      (($# >= 2)) || die "--foxglove-port 缺少参数"
      foxglove_port="$2"
      shift 2
      ;;
    --build-image)
      build_image=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知参数：$1（使用 --help 查看用法）"
      ;;
  esac
done

require_command docker
require_command ip
require_command ping
require_command python3
require_command ss

docker compose version >/dev/null 2>&1 || \
  die "未找到 Docker Compose V2（docker compose）"
[[ -f "${compose_file}" ]] || die "找不到 Compose 文件：${compose_file}"
[[ -e "/sys/class/net/${network_interface}" ]] || \
  die "网卡 ${network_interface} 不存在；使用 'ip -br link' 查看网卡名称"

python3 - "${host_ip}" "${lidar_ip}" "${foxglove_address}" \
  "${foxglove_port}" <<'PY'
import ipaddress
import sys

host = ipaddress.ip_address(sys.argv[1])
lidar = ipaddress.ip_address(sys.argv[2])
bind = ipaddress.ip_address(sys.argv[3])
port = int(sys.argv[4])

if host.version != 4 or lidar.version != 4 or bind.version != 4:
    raise SystemExit('错误：当前脚本的 IP 参数必须是 IPv4 地址')
if host == lidar:
    raise SystemExit('错误：电脑 IP 与雷达 IP 不能相同')
if lidar not in ipaddress.ip_network(f'{host}/24', strict=False):
    raise SystemExit('错误：电脑 IP 与雷达 IP 必须位于同一个 /24 网段')
if not 1 <= port <= 65535:
    raise SystemExit('错误：Foxglove 端口必须在 1 到 65535 之间')
PY

interface_flags="$(cat "/sys/class/net/${network_interface}/flags")"
if (((interface_flags & 1) == 0)); then
  echo "正在启用网卡 ${network_interface}（可能需要输入密码）……"
  run_as_root ip link set dev "${network_interface}" up
  sleep 0.2
fi

carrier="$(cat "/sys/class/net/${network_interface}/carrier" 2>/dev/null || true)"
if [[ "${carrier}" == "0" ]]; then
  die "${network_interface} 没有物理链路，请检查 Mid-360 供电、M12 插头和 RJ45 网线"
fi

host_cidr="${host_ip}/24"
echo "[1/5] 配置雷达网卡 ${network_interface} 为 ${host_cidr}……"
if ip -4 -o address show dev "${network_interface}" | \
  awk '{print $4}' | grep -Fxq "${host_cidr}"; then
  echo "网卡地址已经正确，无需修改。"
elif command -v nmcli >/dev/null 2>&1 && \
  nmcli device modify "${network_interface}" \
    ipv4.addresses "${host_cidr}" ipv4.method manual; then
  echo "已通过 NetworkManager 临时配置网卡。"
else
  echo "NetworkManager 配置失败，改用 sudo ip 配置（可能需要输入密码）。"
  run_as_root ip link set dev "${network_interface}" up
  run_as_root ip address replace "${host_cidr}" dev "${network_interface}"
fi

address_ready=false
for _ in {1..20}; do
  if ip -4 -o address show dev "${network_interface}" | \
    awk '{print $4}' | grep -Fxq "${host_cidr}"; then
    address_ready=true
    break
  fi
  sleep 0.1
done
[[ "${address_ready}" == true ]] || \
  die "未能在 ${network_interface} 上配置 ${host_cidr}"

echo "[2/5] 检查 Mid-360 ${lidar_ip} 的网络连通性……"
ping -I "${network_interface}" -c 2 -W 2 "${lidar_ip}" >/dev/null || \
  die "无法连接 ${lidar_ip}；请检查雷达 IP、供电和网线"
echo "雷达网络正常。"

export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g)}"

if [[ "${build_image}" == true ]] || \
  [[ -z "$(docker compose --project-directory "${project_root}" \
    -f "${compose_file}" images -q algorithm 2>/dev/null)" ]]; then
  echo "[3/5] 构建 algorithm Docker 镜像……"
  docker compose --project-directory "${project_root}" -f "${compose_file}" \
    build algorithm
else
  echo "[3/5] algorithm 镜像已存在，跳过镜像构建。"
fi

echo "[4/5] 启动容器并增量编译地面分割算法……"
docker compose --project-directory "${project_root}" -f "${compose_file}" \
  up -d algorithm

if docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T algorithm pgrep -f \
  "ros2 launch fast_ground_segmenter mid360_ground_segmentation.launch.py" \
  >/dev/null 2>&1; then
  die "Mid-360 启动程序已经在运行；请先在原终端按 Ctrl+C 停止它"
fi

if ss -H -ltn "sport = :${foxglove_port}" 2>/dev/null | grep -q .; then
  die "TCP 端口 ${foxglove_port} 已被占用；请使用 --foxglove-port 指定其他端口"
fi

docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T algorithm bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /opt/livox_ws/install/setup.bash && \
   cd '${container_root}/Fast_Segmentation_ws' && \
   colcon build --symlink-install --packages-select fast_ground_segmenter"

docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T algorithm bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 pkg prefix foxglove_bridge >/dev/null" || \
  die "容器中没有 foxglove_bridge；请使用 --build-image 重建镜像"

echo "[5/5] 启动 Mid-360、地面分割算法和 Foxglove Bridge……"
echo
echo "Foxglove 连接地址：ws://localhost:${foxglove_port}"
echo "3D 面板坐标系：livox_frame"
echo "结果话题：/ground_points（地面）和 /nonground_points（非地面）"
echo "按 Ctrl+C 停止全部实机节点。"
echo

docker compose --project-directory "${project_root}" -f "${compose_file}" \
  exec -T algorithm bash -c \
  "set -Eeo pipefail
   source /opt/ros/jazzy/setup.bash
   source /opt/livox_ws/install/setup.bash
   source '${container_root}/Fast_Segmentation_ws/install/setup.bash'
   set -u
   exec ros2 launch fast_ground_segmenter mid360_ground_segmentation.launch.py \
     host_ip:='${host_ip}' \
     lidar_ip:='${lidar_ip}' \
     foxglove:=true \
     foxglove_address:='${foxglove_address}' \
     foxglove_port:='${foxglove_port}' \
     rviz:=false"
