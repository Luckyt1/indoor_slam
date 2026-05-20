#!/bin/bash
SESSION="indoor_slam"

# 1. 检查并安装 tmux
if ! command -v tmux &> /dev/null; then
    sudo apt update && sudo apt install tmux -y
fi

# 2. 清理旧会话并创建新会话（这步绝对不能省！）
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

# 3. 基础配置
tmux set-option -g mouse on
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " #[fg=black,bg=green] #T #[default] "

# ---------------------------------------------------------------
# 核心布局划分（标准 4 宫格田字格）
# ---------------------------------------------------------------
# 先左右对半切 (生成 0 和 1)
tmux split-window -h -p 50 -t $SESSION

# 把左半边上下对半切 (生成 2，位于左下)
tmux select-pane -t 0
tmux split-window -v -p 50 -t $SESSION

# 把右半边上下对半切 (生成 3，位于右下)
tmux select-pane -t 2
tmux split-window -v -p 50 -t $SESSION

# ---------------------------------------------------------------
# 命名窗格标题 (根据切分逻辑：0=左上, 2=左下, 1=右上, 3=右下)
# ---------------------------------------------------------------
tmux select-pane -t 0 -T "雷达驱动"
tmux select-pane -t 2 -T "里程计"
tmux select-pane -t 1 -T "重定位"
tmux select-pane -t 3 -T "导航"

# ---------------------------------------------------------------
# 发送 ROS2 指令
# ---------------------------------------------------------------
# 0号窗格 (左上): 雷达
tmux send-keys -t $SESSION:0.0 "source install/setup.bash; ros2 launch livox_ros_driver2 msg_MID360s_launch.py" C-m
sleep 2

# 2号窗格 (左下): 里程计 (Point-LIO)
tmux send-keys -t $SESSION:0.2 "source install/setup.bash; ros2 launch point_lio point_lio.launch.py" C-m
sleep 2

# 1号窗格 (右上): 重定位 (small_gicp)
tmux send-keys -t $SESSION:0.1 "source install/setup.bash; ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py" C-m

# 3号窗格 (右下): 导航 (nav)
tmux send-keys -t $SESSION:0.3 "source install/setup.bash; ros2 launch nav indoor_navigation_launch.py" C-m

# 打开会话并聚焦在左上角
tmux select-pane -t 0
tmux attach-session -t $SESSION