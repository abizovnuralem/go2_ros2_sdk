#pragma once

#include <future>
#include <string>
#include <vector>

#include <websocketpp/config/asio_client.hpp>

#include "../parameter.hpp"
#include "../websocket_client.hpp"

namespace foxglove {

std::future<std::vector<uint8_t>> waitForChannelMsg(ClientInterface* client,
                                                    SubscriptionId subscriptionId);

std::future<std::vector<Parameter>> waitForParameters(std::shared_ptr<ClientInterface> client,
                                                      const std::string& requestId = std::string());

std::future<ServiceResponse> waitForServiceResponse(std::shared_ptr<ClientInterface> client);

std::future<Service> waitForService(std::shared_ptr<ClientInterface> client,
                                    const std::string& serviceName);

std::future<Channel> waitForChannel(std::shared_ptr<ClientInterface> client,
                                    const std::string& topicName);

std::future<FetchAssetResponse> waitForFetchAssetResponse(std::shared_ptr<ClientInterface> client);

extern template class Client<websocketpp::config::asio_client>;

}  // namespace foxglove
