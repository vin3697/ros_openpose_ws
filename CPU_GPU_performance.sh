#!/bin/bash

# Create a new tmux session
tmux new-session -d -s monitor_session

# Split the window vertically
tmux split-window -v

# Run nvidia-smi in the first pane within an infinite loop
tmux send-keys -t monitor_session:0.0 'while true; do nvidia-smi; sleep 1; clear; done' Enter

# Run htop in the second pane
tmux send-keys -t monitor_session:0.1 'htop' Enter

# Attach to the tmux session to view the windows
tmux attach-session -t monitor_session
