#!/usr/bin/env bash

set -u

SESSION="indoor_slam"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_MAP="${NAV_MAP:-$WORKSPACE_DIR/src/bxi_nav/maps/maps.yaml}"
RC_ENV_FILE="${BXI_RC_ENV_FILE:-/opt/bxi/bxi_rc_ros2/env.conf}"

# 生产环境由 bxi_rc_ros2 提供唯一的 ROS/DDS 配置。备用 tmux 启动器
# 读取同一份配置，不再创建或硬编码独立 Domain。
if [[ -r "$RC_ENV_FILE" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$RC_ENV_FILE"
    set +a
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"

# ------------------------------------------------------------------
# 基础文件检查
# ------------------------------------------------------------------

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS 2 setup file not found: $ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
    echo "Workspace setup file not found: $WORKSPACE_SETUP" >&2
    echo "Please build the workspace first:" >&2
    echo "  cd \"$WORKSPACE_DIR\" && colcon build" >&2
    exit 1
fi

if [[ ! -f "$NAV_MAP" ]]; then
    echo "Navigation map not found: $NAV_MAP" >&2
    exit 1
fi

# ------------------------------------------------------------------
# 检查 tmux
# ------------------------------------------------------------------

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed."

    sudo apt update || exit 1
    sudo apt install -y tmux || exit 1
fi

# 对路径进行 shell 转义，避免路径中存在空格
printf -v WORKSPACE_SETUP_Q '%q' "$WORKSPACE_SETUP"
printf -v NAV_MAP_Q '%q' "$NAV_MAP"

COMMON_ENV=""
for env_name in ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY CYCLONEDDS_URI; do
    if [[ -v "$env_name" ]]; then
        printf -v env_value_q '%q' "${!env_name}"
        COMMON_ENV+="export ${env_name}=${env_value_q}; "
    fi
done
COMMON_ENV+="source /opt/ros/humble/setup.bash; source $WORKSPACE_SETUP_Q"

# ------------------------------------------------------------------
# 清理旧会话
# ------------------------------------------------------------------

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

# 指定新会话的默认工作目录
tmux new-session \
    -d \
    -s "$SESSION" \
    -c "$WORKSPACE_DIR" \
    -n "indoor_slam"

# ------------------------------------------------------------------
# 当前会话配置
# 使用 -t 指定会话，避免无意影响其他 tmux 会话
# ------------------------------------------------------------------

tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" \
    pane-border-format \
    " #[fg=black,bg=green] #T #[default] "

# ------------------------------------------------------------------
# 创建标准四宫格
#
# 0：左上
# 1：右上
# 2：左下
# 3：右下
# ------------------------------------------------------------------

# 将初始窗格左右切分
tmux split-window \
    -h \
    -p 50 \
    -t "$SESSION:0.0" \
    -c "$WORKSPACE_DIR"

# 切分左侧窗格，创建左下
tmux split-window \
    -v \
    -p 50 \
    -t "$SESSION:0.0" \
    -c "$WORKSPACE_DIR"

# 切分右侧窗格，创建右下
tmux split-window \
    -v \
    -p 50 \
    -t "$SESSION:0.1" \
    -c "$WORKSPACE_DIR"

# 重新均匀排列，确保四个窗格尺寸一致
tmux select-layout -t "$SESSION:0" tiled

# ------------------------------------------------------------------
# 设置窗格标题
# ------------------------------------------------------------------

tmux select-pane -t "$SESSION:0.0" -T "雷达驱动"
tmux select-pane -t "$SESSION:0.1" -T "重定位"
tmux select-pane -t "$SESSION:0.2" -T "里程计"
tmux select-pane -t "$SESSION:0.3" -T "导航"

# ------------------------------------------------------------------
# 启动 ROS 2 节点
# ------------------------------------------------------------------

tmux send-keys \
    -t "$SESSION:0.0" \
    "$COMMON_ENV; ros2 launch livox_ros_driver2 msg_MID360s_launch.py" \
    C-m

sleep 3

tmux send-keys \
    -t "$SESSION:0.2" \
    "$COMMON_ENV; ros2 launch point_lio point_lio_with_mapping_control.launch.py" \
    C-m

sleep 3

tmux send-keys \
    -t "$SESSION:0.1" \
    "$COMMON_ENV; ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py" \
    C-m

sleep 2

tmux send-keys \
    -t "$SESSION:0.3" \
    "$COMMON_ENV; ros2 launch nav indoor_navigation_launch.py map:=$NAV_MAP_Q autostart:=true" \
    C-m

# ------------------------------------------------------------------
# 进入会话
# ------------------------------------------------------------------

tmux select-pane -t "$SESSION:0.0"

if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
