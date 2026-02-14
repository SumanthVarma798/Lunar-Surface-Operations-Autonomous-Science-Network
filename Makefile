.PHONY: dev build clean stop setup test-rover test-earth test-space-link test-telemetry help

# Container-local build path (avoids macOS Docker volume file-locking issues)
BUILD_DIR = /ros_build

help:
	@echo "ROS Humble Development Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make dev              - Open interactive shell in container"
	@echo "  make build            - Build the ROS workspace"
	@echo "  make clean            - Remove build artifacts"
	@echo ""
	@echo "Four-Terminal Workflow (with Space Link relay):"
	@echo "  Terminal 1: make test-space-link  - Space Link relay node"
	@echo "  Terminal 2: make test-rover       - Rover node (on Moon)"
	@echo "  Terminal 3: make test-telemetry   - Telemetry monitor"
	@echo "  Terminal 4: make test-earth       - Earth command station"
	@echo ""
	@echo "  make stop             - Stop and remove container"

setup:
	@if ! docker ps --format '{{.Names}}' | grep -q "^ros-humble-dev$$"; then \
		echo "Starting container..."; \
		docker rm -f ros-humble-dev >/dev/null 2>&1 || true; \
		docker run -dit \
			--name ros-humble-dev \
			ros:humble \
			bash; \
	fi

dev:
	./scripts/rosdev.sh

# Build strategy: use 'docker cp' from the HOST to copy source into the
# container's native filesystem, avoiding macOS Docker volume file-locking
# issues (errno 35 "Resource deadlock avoided"). Build there, then
# 'docker cp' the install artifacts back to the host.
build: setup
	@echo "📦 Copying source into container (via docker cp)..."
	@docker exec ros-humble-dev bash -c "rm -rf $(BUILD_DIR) && mkdir -p $(BUILD_DIR)"
	@docker cp lunar_ops/rover_ws/src ros-humble-dev:$(BUILD_DIR)/src
	@echo "🔨 Building ROS workspace..."
	@docker exec -it ros-humble-dev bash -lc "\
		source /opt/ros/humble/setup.bash && \
		cd $(BUILD_DIR) && \
		colcon build --symlink-install"
	@echo "📋 Syncing install artifacts back to host..."
	@rm -rf lunar_ops/rover_ws/install
	@docker cp ros-humble-dev:$(BUILD_DIR)/install lunar_ops/rover_ws/install
	@echo "✅ Build complete!"

clean:
	rm -rf lunar_ops/rover_ws/build lunar_ops/rover_ws/install lunar_ops/rover_ws/log
	@if docker ps --format '{{.Names}}' | grep -q "^ros-humble-dev$$"; then \
		docker exec ros-humble-dev bash -c "rm -rf $(BUILD_DIR)" 2>/dev/null || true; \
	fi
	@echo "🧹 Clean complete!"

test-space-link: setup build
	@echo "🛰️  Starting Space Link relay node (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "source $(BUILD_DIR)/install/setup.bash && ros2 run rover_core space_link_node"

test-telemetry: setup build
	@echo "📡 Starting Telemetry Monitor (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "source $(BUILD_DIR)/install/setup.bash && ros2 run rover_core telemetry_monitor"

test-rover: setup build
	@echo "🤖 Running rover_node (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "source $(BUILD_DIR)/install/setup.bash && ros2 run rover_core rover_node"

test-earth: setup build
	@echo "🌍 Running earth_node command interface (press Ctrl+C to stop)..."
	docker exec -it ros-humble-dev bash -lc "source $(BUILD_DIR)/install/setup.bash && ros2 run rover_core earth_node"

stop:
	docker rm -f ros-humble-dev || true
