#!/usr/bin/env bash
# Run on Orin. Opens a terminal when launched from a graphical file manager.
set -eo pipefail

monitor_script="$(readlink -f -- "${BASH_SOURCE[0]}")"
monitor_workspace="$(dirname -- "$monitor_script")"

if [[ "${1:-}" == "--in-terminal" ]]; then
    shift
elif [[ ! -t 0 && ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]]; then
    if command -v x-terminal-emulator >/dev/null 2>&1; then
        exec x-terminal-emulator -e bash "$monitor_script" --in-terminal "$@"
    elif command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal -- bash "$monitor_script" --in-terminal "$@"
    fi
    printf '無法開啟 terminal。請在 terminal 執行：bash "%s"\n' "$monitor_script" >&2
    exit 1
fi

monitor_exit() {
    local monitor_status=$?
    trap - EXIT
    if (( monitor_status != 0 && monitor_status != 130 )); then
        printf '\n監控面板啟動或執行失敗（代碼 %s），請查看上方訊息。\n' "$monitor_status" >&2
        printf '若顯示 Address already in use，代表該埠已被占用；請查看原本的面板 terminal，或加上 --port 8089。\n' >&2
    fi
    if [[ -t 0 ]]; then
        printf '\n監控面板已結束。按 Enter 關閉此視窗。'
        read -r _ || true
    fi
    exit "$monitor_status"
}
trap monitor_exit EXIT

if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$monitor_workspace/install/setup.bash" ]]; then
    printf '找不到 ROS Humble 或工作區 install/setup.bash，請先依 docs/rinbo_monitor.md 完成編譯。\n' >&2
    exit 1
fi

cd -- "$monitor_workspace"
source /opt/ros/humble/setup.bash
source "$monitor_workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

printf 'Rinbo 唯讀監控 · ROS domain %s\n' "$ROS_DOMAIN_ID"
printf '請使用下方帶 token 的存取連結；Windows 將 0.0.0.0 換成 Orin 的 IP。\n'
printf 'Orin IP：'
hostname -I || true
printf '保留此視窗；Ctrl+C 只停止監控面板。\n\n'

ros2 run rinbo_monitor rinbo_monitor --host 0.0.0.0 --port 8088 "$@"
