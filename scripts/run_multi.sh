#!/bin/bash

# Check if at least one argument is provided
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <command1> [command2] ..."
  exit 1
fi

# Create a new tmux session without attaching to it
tmux new-session -d -s my_session

# Iterate over each command argument
for i in "$@"; do
  # For the first command, send it to the already created first pane
  if [ "$i" == "$1" ]; then
    tmux send-keys "$i" C-m
  else
    # For subsequent commands, create a new pane and send the command
    tmux split-window -h
    tmux select-layout even-horizontal
    tmux send-keys "$i" C-m
  fi
done

# Attach to the tmux session
tmux attach-session -t my_session
