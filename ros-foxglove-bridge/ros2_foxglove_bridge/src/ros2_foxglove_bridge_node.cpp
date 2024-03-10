#include <rclcpp_components/component_manager.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  size_t numThreads = 0;
  {
    // Temporary dummy node to get num_threads param.
    auto dummyNode = std::make_shared<rclcpp::Node>("dummy");
    auto numThreadsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    numThreadsDescription.name = "num_threads";
    numThreadsDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    numThreadsDescription.description =
      "The number of threads to use for the ROS node executor. 0 means one thread per CPU core.";
    numThreadsDescription.read_only = true;
    numThreadsDescription.additional_constraints = "Must be a non-negative integer";
    numThreadsDescription.integer_range.resize(1);
    numThreadsDescription.integer_range[0].from_value = 0;
    numThreadsDescription.integer_range[0].to_value = INT32_MAX;
    numThreadsDescription.integer_range[0].step = 1;
    constexpr int DEFAULT_NUM_THREADS = 0;
    dummyNode->declare_parameter(numThreadsDescription.name, DEFAULT_NUM_THREADS,
                                 numThreadsDescription);
    numThreads = static_cast<size_t>(dummyNode->get_parameter(numThreadsDescription.name).as_int());
  }

  auto executor =
    rclcpp::executors::MultiThreadedExecutor::make_shared(rclcpp::ExecutorOptions{}, numThreads);

  rclcpp_components::ComponentManager componentManager(executor,
                                                       "foxglove_bridge_component_manager");
  const auto componentResources = componentManager.get_component_resources("foxglove_bridge");

  if (componentResources.empty()) {
    RCLCPP_INFO(componentManager.get_logger(), "No loadable resources found");
    return EXIT_FAILURE;
  }

  auto componentFactory = componentManager.create_component_factory(componentResources.front());
  auto node = componentFactory->create_node_instance(rclcpp::NodeOptions());

  executor->add_node(node.get_node_base_interface());
  executor->spin();
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
