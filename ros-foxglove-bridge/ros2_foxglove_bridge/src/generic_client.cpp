#include <future>
#include <iostream>

#include <rclcpp/client.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <foxglove_bridge/generic_client.hpp>

// clang-format off
/* True if the version of RCLCPP is at least major.minor.patch */
#define RCLCPP_VERSION_GTE(major, minor, patch)        \
  (major < RCLCPP_VERSION_MAJOR                        \
     ? true                                            \
     : major > RCLCPP_VERSION_MAJOR                    \
         ? false                                       \
         : minor < RCLCPP_VERSION_MINOR                \
             ? true                                    \
             : minor > RCLCPP_VERSION_MINOR            \
                 ? false                               \
                 : patch < RCLCPP_VERSION_PATCH ? true \
                                                : patch > RCLCPP_VERSION_PATCH ? false : true)
// clang-format on

namespace {

// Copy of github.com/ros2/rclcpp/blob/33dae5d67/rclcpp/src/rclcpp/typesupport_helpers.cpp#L69-L92
static std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string& full_type) {
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos || sep_position_back == 0 ||
      sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
      "Message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}
}  // namespace

namespace foxglove_bridge {

constexpr char TYPESUPPORT_INTROSPECTION_LIB_NAME[] = "rosidl_typesupport_introspection_cpp";
constexpr char TYPESUPPORT_LIB_NAME[] = "rosidl_typesupport_cpp";
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ServiceMembers;

std::shared_ptr<void> allocate_message(const MessageMembers* members) {
  void* buffer = malloc(members->size_of_);
  if (buffer == nullptr) {
    throw std::runtime_error("Failed to allocate memory");
  }
  memset(buffer, 0, members->size_of_);
  members->init_function(buffer, rosidl_runtime_cpp::MessageInitialization::ALL);
  return std::shared_ptr<void>(buffer, free);
}

std::string getTypeIntrospectionSymbolName(const std::string& serviceType) {
  const auto [pkgName, middleModule, typeName] = extract_type_identifier(serviceType);

  return std::string(TYPESUPPORT_INTROSPECTION_LIB_NAME) + "__get_service_type_support_handle__" +
         pkgName + "__" + (middleModule.empty() ? "srv" : middleModule) + "__" + typeName;
}

/**
 * The default symbol names for getting type support handles for services are missing from the
 * rosidl_typesupport_cpp shared libraries, see
 * https://github.com/ros2/rosidl_typesupport/issues/122
 *
 * We can however, as a (hacky) workaround, use other symbols defined in the shared library.
 * With `nm -C -D /opt/ros/humble/lib/libtest_msgs__rosidl_typesupport_cpp.so` we see that there is
 * `rosidl_service_type_support_t const*
 * rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>()` which
 * mangled becomes
 * `_ZN22rosidl_typesupport_cpp31get_service_type_support_handleIN9test_msgs3srv10BasicTypesEEEPK29rosidl_service_type_support_tv`
 * This is the same for galactic, humble and rolling (tested with gcc / clang)
 *
 * This function produces the mangled symbol name for a given service type.
 *
 * \param[in] serviceType The service type, e.g. "test_msgs/srv/BasicTypes"
 * \return Symbol name for getting the service type support handle
 */
std::string getServiceTypeSupportHandleSymbolName(const std::string& serviceType) {
  const auto [pkgName, middleModule, typeName] = extract_type_identifier(serviceType);
  const auto lengthPrefixedString = [](const std::string& s) {
    return std::to_string(s.size()) + s;
  };

  return "_ZN" + lengthPrefixedString(TYPESUPPORT_LIB_NAME) +
         lengthPrefixedString("get_service_type_support_handle") + "IN" +
         lengthPrefixedString(pkgName) +
         lengthPrefixedString(middleModule.empty() ? "srv" : middleModule) +
         lengthPrefixedString(typeName) + "EEEPK" +
         lengthPrefixedString("rosidl_service_type_support_t") + "v";
}

GenericClient::GenericClient(rclcpp::node_interfaces::NodeBaseInterface* nodeBase,
                             rclcpp::node_interfaces::NodeGraphInterface::SharedPtr nodeGraph,
                             std::string serviceName, std::string serviceType,
                             rcl_client_options_t& client_options)
    : rclcpp::ClientBase(nodeBase, nodeGraph) {
  const auto [pkgName, middleModule, typeName] = extract_type_identifier(serviceType);
  const auto requestTypeName = serviceType + "_Request";
  const auto responseTypeName = serviceType + "_Response";

  _typeSupportLib = rclcpp::get_typesupport_library(serviceType, TYPESUPPORT_LIB_NAME);
  _typeIntrospectionLib =
    rclcpp::get_typesupport_library(serviceType, TYPESUPPORT_INTROSPECTION_LIB_NAME);
  if (!_typeSupportLib || !_typeIntrospectionLib) {
    throw std::runtime_error("Failed to load shared library for service type " + serviceType);
  }

  const auto typesupportSymbolName = getServiceTypeSupportHandleSymbolName(serviceType);
  if (!_typeSupportLib->has_symbol(typesupportSymbolName)) {
    throw std::runtime_error("Failed to find symbol '" + typesupportSymbolName + "' in " +
                             _typeSupportLib->get_library_path());
  }

  const rosidl_service_type_support_t* (*get_ts)() = nullptr;
  _serviceTypeSupportHdl =
    (reinterpret_cast<decltype(get_ts)>(_typeSupportLib->get_symbol(typesupportSymbolName)))();

  const auto typeinstrospection_symbol_name = getTypeIntrospectionSymbolName(serviceType);

  // This will throw runtime_error if the symbol was not found.
  _typeIntrospectionHdl = (reinterpret_cast<decltype(get_ts)>(
    _typeIntrospectionLib->get_symbol(typeinstrospection_symbol_name)))();

  // get_typesupport_handle is deprecated since rclcpp 25.0.0
  // (https://github.com/ros2/rclcpp/pull/2209)
#if RCLCPP_VERSION_GTE(25, 0, 0)
  _requestTypeSupportHdl =
    rclcpp::get_message_typesupport_handle(requestTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
  _responseTypeSupportHdl = rclcpp::get_message_typesupport_handle(
    responseTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
#else
  _requestTypeSupportHdl =
    rclcpp::get_typesupport_handle(requestTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
  _responseTypeSupportHdl =
    rclcpp::get_typesupport_handle(responseTypeName, TYPESUPPORT_LIB_NAME, *_typeSupportLib);
#endif

  rcl_ret_t ret = rcl_client_init(this->get_client_handle().get(), this->get_rcl_node_handle(),
                                  _serviceTypeSupportHdl, serviceName.c_str(), &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(serviceName, rcl_node_get_name(rcl_node_handle),
                                           rcl_node_get_namespace(rcl_node_handle), true);
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
  }
}

std::shared_ptr<void> GenericClient::create_response() {
  auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
  return allocate_message(srv_members->response_members_);
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header() {
  return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
}

void GenericClient::handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<void> response) {
  std::unique_lock<std::mutex> lock(pending_requests_mutex_);
  int64_t sequence_number = request_header->sequence_number;

  auto ser_response = std::make_shared<rclcpp::SerializedMessage>();
  rmw_ret_t r = rmw_serialize(response.get(), _responseTypeSupportHdl,
                              &ser_response->get_rcl_serialized_message());
  if (r != RMW_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("foxglove_bridge", "Failed to serialize service response. Ignoring...");
    return;
  }

  // TODO(esteve) this should throw instead since it is not expected to happen in the first place
  if (this->pending_requests_.count(sequence_number) == 0) {
    RCUTILS_LOG_ERROR_NAMED("foxglove_bridge", "Received invalid sequence number. Ignoring...");
    return;
  }
  auto tuple = this->pending_requests_[sequence_number];
  auto call_promise = std::get<0>(tuple);
  auto callback = std::get<1>(tuple);
  auto future = std::get<2>(tuple);
  this->pending_requests_.erase(sequence_number);
  // Unlock here to allow the service to be called recursively from one of its callbacks.
  lock.unlock();

  call_promise->set_value(ser_response);
  callback(future);
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request) {
  return async_send_request(request, [](SharedFuture) {});
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request,
                                                              CallbackType&& cb) {
  std::lock_guard<std::mutex> lock(pending_requests_mutex_);
  int64_t sequence_number;

  auto srv_members = static_cast<const ServiceMembers*>(_typeIntrospectionHdl->data);
  auto buf = allocate_message(srv_members->request_members_);

  const rmw_serialized_message_t* sm = &request->get_rcl_serialized_message();
  if (const auto ret = rmw_deserialize(sm, _requestTypeSupportHdl, buf.get()) != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to desirialize request");
  }
  rcl_ret_t ret = rcl_send_request(get_client_handle().get(), buf.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
  }

  SharedPromise call_promise = std::make_shared<Promise>();
  SharedFuture f(call_promise->get_future());
  pending_requests_[sequence_number] =
    std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
  return f;
}

}  // namespace foxglove_bridge
