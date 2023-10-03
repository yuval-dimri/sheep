# Makefile for ROS2

LOCK_FILE := .lock

# Default target
all: build

# Build the ROS2 code
build: clean
	@echo "Building ROS2 code..."
	colcon build --symlink-install

# Run the simulation setup
simulation:  build
	ros2 launch sheep-sim sim.launch.py

# Run the robot code
robot: 
	ros2 run sheep-hardware hw.launch.py

# Add other targets as needed
other:
	@echo "Running other stuff..."
	# Add other commands here

# Clean the build and install directories
clean:
	@echo "Cleaning up..."
	rm -rf build/ install/ log/

.PHONY: all build simulation robot check_lock other clean
