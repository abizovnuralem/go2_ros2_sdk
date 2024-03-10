#pragma once

#include <future>

#include <rclcpp/client.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcpputils/shared_library.hpp>

namespace foxglove_bridge {

class GenericClient : public rclcpp::ClientBase {
public:
  using SharedRequest = std::shared_ptr<rclcpp::SerializedMessage>;
  using SharedResponse = std::shared_ptr<rclcpp::SerializedMessage>;
  using Promise = std::promise<SharedResponse>;
  using PromiseWithRequest = std::promise<std::pair<SharedRequest, SharedResponse>>;
  using SharedPromise = std::shared_ptr<Promise>;
  using SharedPromiseWithRequest = std::shared_ptr<PromiseWithRequest>;
  using SharedFuture = std::shared_future<SharedResponse>;
  using SharedFutureWithRequest = std::shared_future<std::pair<SharedRequest, SharedResponse>>;
  using CallbackType = std::function<void(SharedFuture)>;
  using CallbackWithRequestType = std::function<void(SharedFutureWithRequest)>;

  RCLCPP_SMART_PTR_DEFINITIONS(GenericClient)

  GenericClient(rclcpp::node_interfaces::NodeBaseInterface* node_base,
                rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                std::string service_name, std::string service_type,
                rcl_client_options_t& client_options);
  virtual ~GenericClient() {}

  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                       std::shared_ptr<void> response) override;
  SharedFuture async_send_request(SharedRequest request);
  SharedFuture async_send_request(SharedRequest request, CallbackType&& cb);

private:
  RCLCPP_DISABLE_COPY(GenericClient)

  std::map<int64_t, std::tuple<SharedPromise, CallbackType, SharedFuture>> pending_requests_;
  std::mutex pending_requests_mutex_;
  std::shared_ptr<rcpputils::SharedLibrary> _typeSupportLib;
  std::shared_ptr<rcpputils::SharedLibrary> _typeIntrospectionLib;
  const rosidl_service_type_support_t* _serviceTypeSupportHdl;
  const rosidl_message_type_support_t* _requestTypeSupportHdl;
  const rosidl_message_type_support_t* _responseTypeSupportHdl;
  const rosidl_service_type_support_t* _typeIntrospectionHdl;
};

}  // namespace foxglove_bridge
