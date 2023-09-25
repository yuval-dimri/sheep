# Makefile for ROS2

LOCK_FILE := .lock

# Default target
all: build

# Build the ROS2 code
build:
	@echo "Building ROS2 code..."
	colcon build --symlink-install

# Run the simulation setup
simulation: check_lock
	@echo "Running simulation setup..."
	touch $(LOCK_FILE)
	trap 'rm -f $(LOCK_FILE); exit' INT; \
	ros2 launch sheep-sim sim.launch.py
	rm -f $(LOCK_FILE)

# Run the robot code
robot: check_lock
	@echo "Running robot code..."
	touch $(LOCK_FILE)
	trap 'rm -f $(LOCK_FILE); exit' INT; \
	ros2 run sheep-hardware wh.launch.py
	rm -f $(LOCK_FILE)

# Check for lock file
check_lock:
	@if [ -f $(LOCK_FILE) ]; then \
		echo "Another instance (simulation or robot) is already running. Stop it first."; \
		exit 1; \
	fi

# Add other targets as needed
other:
	@echo "Running other stuff..."
	# Add other commands here

# Clean the build and install directories
clean:
	@echo "Cleaning up..."
	rm -rf build/ install/ log/

.PHONY: all build simulation robot check_lock other clean
