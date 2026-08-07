#!/bin/bash
set -euo pipefail

SESSION="indoor_slam"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 与 start.sh 一致: 匹配与安装位置无关, 否则脚本目录 ≠ 安装根时 SIGINT
# 打不中 manager, tmux kill-session 直接 SIGHUP 会把 start_new_session 的
# 子进程组 (Livox/Point-LIO/GICP/Nav2) 全部留成孤儿。
manager_pattern="lib/bxi_slam_manager/bxi_slam_manager"

# 子进程 (Livox/Point-LIO/GICP/Nav2) 是 start_new_session=True 启动的独立会话,
# 直接 kill tmux 只会 SIGHUP 掉主管, 跳过 destroy_node 清理, 把整条流水线留成
# 孤儿。先 SIGINT 主管让它走 idle 收尾 (停全部子进程组), 超时再强杀。
if pgrep -f -- "$manager_pattern" >/dev/null 2>&1; then
    pkill -INT -f -- "$manager_pattern" 2>/dev/null || true
    for _ in $(seq 1 20); do
        pgrep -f -- "$manager_pattern" >/dev/null 2>&1 || break
        sleep 0.5
    done
    if pgrep -f -- "$manager_pattern" >/dev/null 2>&1; then
        echo "SLAM supervisor did not exit gracefully; killing." >&2
        pkill -KILL -f -- "$manager_pattern" 2>/dev/null || true
    fi
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
