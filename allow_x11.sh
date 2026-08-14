#!/usr/bin/env bash

set -Eeuo pipefail

if ! command -v xhost >/dev/null 2>&1; then
  echo "错误：宿主机未安装 xhost。请先运行：sudo apt install x11-xserver-utils" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "错误：宿主机 DISPLAY 为空，请在图形桌面的终端中运行本脚本。" >&2
  exit 1
fi

host_user="$(id -un)"

case "${1:-allow}" in
  allow)
    xhost "+SI:localuser:${host_user}"
    echo "已允许本地用户 ${host_user} 访问显示 ${DISPLAY}。"
    ;;
  revoke)
    xhost "-SI:localuser:${host_user}"
    echo "已撤销本地用户 ${host_user} 的显示访问权限。"
    ;;
  *)
    echo "用法：$0 [allow|revoke]" >&2
    exit 2
    ;;
esac
