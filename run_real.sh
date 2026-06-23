#!/bin/bash

if [ -n "$TMUX" ]; then
    echo "Running in TMUX not supported to avoid having to deal with potential complications"
    exit 1
fi

set -e

if tmux has-session -t "gopro-process"; then
    echo "GoPro server is running. You can attach to it using 'tmux attach-session -t gopro-process'"
else
    echo "No GoPro server is running. Starting a new session..."
    tmux new-session -d -s "gopro-process"
    tmux send-keys -t "gopro-process:gopro-webcam" "echo \"Enter your password, then press Ctrl-B, then d to exit\"" C-m
    # taken from the launch script in the GoPro-ROS2 node (scripts/webcam_service.sh)
    tmux new-window -d -t gopro-process -n gopro-webcam
    tmux send-keys -t "gopro-process:gopro-webcam" "sudo gopro webcam -n -p enp*" C-m
    tmux attach-session -t "gopro-process:gopro-webcam"
    if [[ "$(tmux capture-pane -p -t "session_name:window.pane" | grep -q "Error while starting the Webcam mode")" == "0" ]]; then
        echo "GoPro server may have failed to start. Check it out at 'tmux attach-session -t gopro-process'"
        exit 1
    fi
    tmux new-window -d -t gopro-process -n ffmpeg-stream
    tmux send-keys -t "gopro-process:ffmpeg-stream" "ffmpeg -nostdin -threads 1 -i 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=50000000' -f:v mpegts -fflags nobuffer -vf format=yuv420p -f v4l2 /dev/video42" C-m
    echo "GoPro server (and ffmpeg) is now running. You can attach to it using 'tmux attach-session -t gopro-process'"
fi

# kill any existing session that exists
tmux kill-session -t ros2-xarm 2>/dev/null || true; tmux new -d -s ros2-xarm

SOURCE_COMMANDS="source /opt/ros/humble/setup.bash && source install/setup.bash"

# launch gopro node
tmux new-window -d -t ros2-xarm -n "gopro"
tmux send-keys -t "ros2-xarm:gopro" "$SOURCE_COMMANDS" C-m
tmux send-keys -t "ros2-xarm:gopro" "ros2 launch camera_cpp go_pro_launch.py" C-m

# launch planning node
tmux new-window -d -t ros2-xarm -n "planning"
tmux send-keys -t "ros2-xarm:planning" "$SOURCE_COMMANDS" C-m
tmux send-keys -t "ros2-xarm:planning" "ros2 launch xarm6_pointer planning_env.launch.py is_live:=true robot_ip:=192.168.1.213" C-m

# launch control node
tmux new-window -d -t ros2-xarm -n "controller"
tmux send-keys -t "ros2-xarm:controller" "$SOURCE_COMMANDS" C-m
tmux send-keys -t "ros2-xarm:controller" "ros2 launch xarm6_pointer pointer_node.launch.py controllers_name:=controllers" C-m

# launch weed detection node
tmux new-window -d -t ros2-xarm -n "weed"
tmux send-keys -t "ros2-xarm:weed" "$SOURCE_COMMANDS" C-m
tmux send-keys -t "ros2-xarm:weed" "ros2 launch weed_detection weed_detection.launch.py" C-m

echo "Everything should be running, you can view startup logs in windows in 'tmux attach-session -t ros2-xarm' (Ctrl-B p/n to switch windows)"
