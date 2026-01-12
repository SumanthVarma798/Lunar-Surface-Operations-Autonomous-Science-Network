.PHONY: dev build clean stop setup test-rover test-earth help

help:
	@echo "ROS Humble Development Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make dev         - Open interactive shell in container"
	@echo "  make build       - Build the ROS workspace"
	@echo "  make clean       - Remove build artifacts"
	@echo "  make test-rover  - Run rover_node (Ctrl+C to stop)"
	@echo "  make test-earth  - Run earth_node (Ctrl+C to stop)"
	@echo "  make stop        - Stop and remove container"

setup:
	@if ! docker ps --format '{{.Names}}' | grep -q "^ros-humble-dev$$"; then \
		echo "Starting container..."; \
		docker rm -f ros-humble-dev >/dev/null 2>&1 || true; \
		docker run -dit \
			--name ros-humble-dev \
			-v "$$(pwd):/workspace" \
			-w /workspace/lunar_ops/rover_ws \
			ros:humble \
			bash; \
	fi

dev:
	./scripts/rosdev.sh

build: setup
	docker exec -it ros-humble-dev bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/lunar_ops/rover_ws && colcon build --symlink-install"
	@echo "Creating executable symlinks for ros2 run..."
	@docker exec ros-humble-dev bash -c "cd /workspace/lunar_ops/rover_ws && mkdir -p install/rover_core/lib/rover_core && ln -sf ../../bin/rover_node install/rover_core/lib/rover_core/rover_node && ln -sf ../../bin/earth_node install/rover_core/lib/rover_core/earth_node"

clean: setup
	docker exec -it ros-humble-dev bash -lc "cd /workspace/lunar_ops/rover_ws && rm -rf build install log"

test-rover: setup build
	@echo "Running rover_node (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "cd /workspace/lunar_ops/rover_ws && source install/setup.bash && ros2 run rover_core rover_node"

test-earth: setup build
	@echo "Running earth_node (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "cd /workspace/lunar_ops/rover_ws && source install/setup.bash && ros2 run rover_core earth_node"

stop:
	docker rm -f ros-humble-dev || true
