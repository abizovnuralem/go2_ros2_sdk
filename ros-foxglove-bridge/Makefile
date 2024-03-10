ROS1_DISTRIBUTIONS := melodic noetic
ROS2_DISTRIBUTIONS := humble iron rolling

define generate_ros1_targets
.PHONY: $(1)
$(1):
	docker build -t foxglove_bridge_$(1) --pull -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=$(1) .

.PHONY: $(1)-test
$(1)-test: $(1)
	docker run -t --rm foxglove_bridge_$(1) bash -c "catkin_make run_tests && catkin_test_results"

.PHONY: $(1)-boost-asio
$(1)-boost-asio:
	docker build -t foxglove_bridge_$(1)_boost_asio --pull -f Dockerfile.ros1 --build-arg ROS_DISTRIBUTION=$(1) --build-arg USE_ASIO_STANDALONE=OFF .

.PHONY: $(1)-test-boost-asio
$(1)-test-boost-asio: $(1)-boost-asio
	docker run -t --rm foxglove_bridge_$(1)_boost_asio bash -c "catkin_make run_tests && catkin_test_results"
endef

define generate_ros2_targets
.PHONY: $(1)
$(1):
	docker build -t foxglove_bridge_$(1) --pull -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=$(1) .

.PHONY: $(1)-test
$(1)-test: $(1)
	docker run -t --rm foxglove_bridge_$(1) colcon test --event-handlers console_cohesion+ --return-code-on-test-failure

.PHONY: $(1)-boost-asio
$(1)-boost-asio:
	docker build -t foxglove_bridge_$(1)-boost-asio --pull -f Dockerfile.ros2 --build-arg ROS_DISTRIBUTION=$(1) --build-arg USE_ASIO_STANDALONE=OFF .

.PHONY: $(1)-test-boost-asio
$(1)-test-boost-asio: $(1)-boost-asio
	docker run -t --rm foxglove_bridge_$(1)-boost-asio colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
endef

$(foreach distribution,$(ROS1_DISTRIBUTIONS),$(eval $(call generate_ros1_targets,$(strip $(distribution)))))
$(foreach distribution,$(ROS2_DISTRIBUTIONS),$(eval $(call generate_ros2_targets,$(strip $(distribution)))))


default: ros2

.PHONY: ros1
ros1:
	docker build -t foxglove_bridge_ros1 --pull -f Dockerfile.ros1 .

.PHONY: ros2
ros2:
	docker build -t foxglove_bridge_ros2 --pull -f Dockerfile.ros2 .

.PHONY: rosdev
rosdev:
	docker build -t foxglove_bridge_rosdev --pull -f .devcontainer/Dockerfile .

clean:
	docker rmi $(docker images --filter=reference="foxglove_bridge_*" -q)

.PHONY: lint
lint: rosdev
	docker run -t --rm -v $(CURDIR):/src foxglove_bridge_rosdev python3 /src/scripts/format.py /src
