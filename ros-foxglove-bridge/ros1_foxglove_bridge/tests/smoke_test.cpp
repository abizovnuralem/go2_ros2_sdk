#include <chrono>
#include <future>
#include <thread>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/builtin_string.h>
#include <std_srvs/SetBool.h>
#include <websocketpp/config/asio_client.hpp>

#include <foxglove_bridge/test/test_client.hpp>
#include <foxglove_bridge/websocket_client.hpp>

constexpr char URI[] = "ws://localhost:9876";

// Binary representation of std_msgs/String for "hello world"
constexpr uint8_t HELLO_WORLD_BINARY[] = {11,  0,  0,   0,   104, 101, 108, 108,
                                          111, 32, 119, 111, 114, 108, 100};

constexpr auto ONE_SECOND = std::chrono::seconds(1);
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(8);

class ParameterTest : public ::testing::Test {
public:
  using PARAM_1_TYPE = std::string;
  inline static const std::string PARAM_1_NAME = "/node_1/string_param";
  inline static const PARAM_1_TYPE PARAM_1_DEFAULT_VALUE = "hello";

  using PARAM_2_TYPE = std::vector<double>;
  inline static const std::string PARAM_2_NAME = "/node_2/int_array_param";
  inline static const PARAM_2_TYPE PARAM_2_DEFAULT_VALUE = {1.2, 2.1, 3.3};

protected:
  void SetUp() override {
    _nh = ros::NodeHandle();
    _nh.setParam(PARAM_1_NAME, PARAM_1_DEFAULT_VALUE);
    _nh.setParam(PARAM_2_NAME, PARAM_2_DEFAULT_VALUE);

    _wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
    ASSERT_EQ(std::future_status::ready, _wsClient->connect(URI).wait_for(DEFAULT_TIMEOUT));
  }

  ros::NodeHandle _nh;
  std::shared_ptr<foxglove::Client<websocketpp::config::asio_client>> _wsClient;
};

class ServiceTest : public ::testing::Test {
public:
  inline static const std::string SERVICE_NAME = "/foo_service";

protected:
  void SetUp() override {
    _nh = ros::NodeHandle();
    _service = _nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
      SERVICE_NAME, [&](auto& req, auto& res) {
        res.message = "hello";
        res.success = req.data;
        return true;
      });
  }

private:
  ros::NodeHandle _nh;
  ros::ServiceServer _service;
};

TEST(SmokeTest, testConnection) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  EXPECT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(DEFAULT_TIMEOUT));
}

TEST(SmokeTest, testSubscription) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::String>(topic_name, 10, true);
  pub.publish(std::string("hello world"));

  // Connect a few clients and make sure that they receive the correct message
  const auto clientCount = 3;
  for (auto i = 0; i < clientCount; ++i) {
    // Set up a client and subscribe to the channel.
    auto client = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
    auto channelFuture = foxglove::waitForChannel(client, topic_name);
    ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(ONE_SECOND));
    ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DEFAULT_TIMEOUT));
    const foxglove::Channel channel = channelFuture.get();
    const foxglove::SubscriptionId subscriptionId = 1;

    // Subscribe to the channel and confirm that the promise resolves
    auto msgFuture = waitForChannelMsg(client.get(), subscriptionId);
    client->subscribe({{subscriptionId, channel.id}});
    ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(ONE_SECOND));
    const auto msgData = msgFuture.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0, std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));

    // Unsubscribe from the channel again.
    client->unsubscribe({subscriptionId});
  }
}

TEST(SmokeTest, testSubscriptionParallel) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::String>(topic_name, 10, true);
  pub.publish(std::string("hello world"));

  // Connect a few clients (in parallel) and make sure that they receive the correct message
  const foxglove::SubscriptionId subscriptionId = 1;
  auto clients = {
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
  };

  std::vector<std::future<std::vector<uint8_t>>> futures;
  for (auto client : clients) {
    futures.push_back(waitForChannelMsg(client.get(), subscriptionId));
  }

  for (auto client : clients) {
    auto channelFuture = foxglove::waitForChannel(client, topic_name);
    ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(ONE_SECOND));
    ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(DEFAULT_TIMEOUT));
    const foxglove::Channel channel = channelFuture.get();
    client->subscribe({{subscriptionId, channel.id}});
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
    auto msgData = future.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0, std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));
  }

  for (auto client : clients) {
    client->unsubscribe({subscriptionId});
  }
}

TEST(SmokeTest, testPublishing) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;

  foxglove::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/foo";
  advertisement.encoding = "ros1";
  advertisement.schemaName = "std_msgs/String";

  // Set up a ROS node with a subscriber
  ros::NodeHandle nh;
  std::promise<std::string> msgPromise;
  auto msgFuture = msgPromise.get_future();
  auto subscriber = nh.subscribe<std_msgs::String>(
    advertisement.topic, 10, [&msgPromise](const std_msgs::String::ConstPtr& msg) {
      msgPromise.set_value(msg->data);
    });

  // Set up the client, advertise and publish the binary message
  ASSERT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(DEFAULT_TIMEOUT));
  wsClient.advertise({advertisement});
  std::this_thread::sleep_for(ONE_SECOND);
  wsClient.publish(advertisement.channelId, HELLO_WORLD_BINARY, sizeof(HELLO_WORLD_BINARY));
  wsClient.unadvertise({advertisement.channelId});

  // Ensure that we have received the correct message via our ROS subscriber
  const auto msgResult = msgFuture.wait_for(ONE_SECOND);
  ASSERT_EQ(std::future_status::ready, msgResult);
  EXPECT_EQ("hello world", msgFuture.get());
}

TEST_F(ParameterTest, testGetAllParams) {
  const std::string requestId = "req-testGetAllParams";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->getParameters({}, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_GE(params.size(), 2UL);
}

TEST_F(ParameterTest, testGetNonExistingParameters) {
  const std::string requestId = "req-testGetNonExistingParameters";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->getParameters(
    {"/foo_1/non_existing_parameter", "/foo_2/non_existing/nested_parameter"}, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_TRUE(params.empty());
}

TEST_F(ParameterTest, testGetParameters) {
  const std::string requestId = "req-testGetParameters";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->getParameters({PARAM_1_NAME, PARAM_2_NAME}, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_1_NAME;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_2_NAME;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(PARAM_1_DEFAULT_VALUE, p1Iter->getValue().getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());

  std::vector<double> double_array_val;
  const auto array_params = p2Iter->getValue().getValue<std::vector<foxglove::ParameterValue>>();
  for (const auto& paramValue : array_params) {
    double_array_val.push_back(paramValue.getValue<double>());
  }
  EXPECT_EQ(double_array_val, PARAM_2_DEFAULT_VALUE);
}

TEST_F(ParameterTest, testSetParameters) {
  const PARAM_1_TYPE newP1value = "world";
  const std::vector<foxglove::ParameterValue> newP2value = {4.1, 5.5, 6.6};

  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(PARAM_1_NAME, newP1value),
    foxglove::Parameter(PARAM_2_NAME, newP2value),
  };

  _wsClient->setParameters(parameters);
  const std::string requestId = "req-testSetParameters";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->getParameters({PARAM_1_NAME, PARAM_2_NAME}, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_1_NAME;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_2_NAME;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(newP1value, p1Iter->getValue().getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());

  std::vector<double> double_array_val;
  const auto array_params = p2Iter->getValue().getValue<std::vector<foxglove::ParameterValue>>();
  for (const auto& paramValue : array_params) {
    double_array_val.push_back(paramValue.getValue<double>());
  }
  const std::vector<double> expected_value = {4.1, 5.5, 6.6};
  EXPECT_EQ(double_array_val, expected_value);
}

TEST_F(ParameterTest, testSetParametersWithReqId) {
  const PARAM_1_TYPE newP1value = "world";
  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(PARAM_1_NAME, newP1value),
  };

  const std::string requestId = "req-testSetParameters";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->setParameters(parameters, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_EQ(1UL, params.size());
}

TEST_F(ParameterTest, testUnsetParameter) {
  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(PARAM_1_NAME),
  };

  const std::string requestId = "req-testUnsetParameter";
  auto future = foxglove::waitForParameters(_wsClient, requestId);
  _wsClient->setParameters(parameters, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  EXPECT_EQ(0UL, params.size());
}

TEST_F(ParameterTest, testParameterSubscription) {
  auto future = foxglove::waitForParameters(_wsClient);

  _wsClient->subscribeParameterUpdates({PARAM_1_NAME});
  _wsClient->setParameters({foxglove::Parameter(PARAM_1_NAME, "foo")});
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  std::vector<foxglove::Parameter> params = future.get();

  ASSERT_EQ(1UL, params.size());
  EXPECT_EQ(params.front().getName(), PARAM_1_NAME);

  _wsClient->unsubscribeParameterUpdates({PARAM_1_NAME});
  _wsClient->setParameters({foxglove::Parameter(PARAM_1_NAME, "bar")});

  future = foxglove::waitForParameters(_wsClient);
  ASSERT_EQ(std::future_status::timeout, future.wait_for(ONE_SECOND));
}

TEST_F(ParameterTest, testGetParametersParallel) {
  // Connect a few clients (in parallel) and make sure that they all receive parameters
  auto clients = {
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
  };

  std::vector<std::future<std::vector<foxglove::Parameter>>> futures;
  for (auto client : clients) {
    futures.push_back(
      std::async(std::launch::async, [client]() -> std::vector<foxglove::Parameter> {
        if (std::future_status::ready == client->connect(URI).wait_for(DEFAULT_TIMEOUT)) {
          const std::string requestId = "req-123";
          auto future = foxglove::waitForParameters(client, requestId);
          client->getParameters({}, requestId);
          future.wait_for(DEFAULT_TIMEOUT);
          if (future.valid()) {
            return future.get();
          }
        }
        return {};
      }));
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
    std::vector<foxglove::Parameter> parameters;
    EXPECT_NO_THROW(parameters = future.get());
    EXPECT_GE(parameters.size(), 2UL);
  }
}

TEST_F(ServiceTest, testCallServiceParallel) {
  // Connect a few clients (in parallel) and make sure that they can all call the service
  auto clients = {
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
  };

  auto serviceFuture = foxglove::waitForService(*clients.begin(), SERVICE_NAME);
  for (auto client : clients) {
    ASSERT_EQ(std::future_status::ready, client->connect(URI).wait_for(std::chrono::seconds(5)));
  }
  ASSERT_EQ(std::future_status::ready, serviceFuture.wait_for(std::chrono::seconds(5)));
  const foxglove::Service service = serviceFuture.get();

  foxglove::ServiceRequest request;
  request.serviceId = service.id;
  request.callId = 123lu;
  request.encoding = "ros1";
  request.data = {1};  // Serialized boolean "True"

  const std::vector<uint8_t> expectedSerializedResponse = {1, 5, 0, 0, 0, 104, 101, 108, 108, 111};

  std::vector<std::future<foxglove::ServiceResponse>> futures;
  for (auto client : clients) {
    futures.push_back(foxglove::waitForServiceResponse(client));
    client->sendServiceRequest(request);
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(5)));
    foxglove::ServiceResponse response;
    EXPECT_NO_THROW(response = future.get());
    EXPECT_EQ(response.serviceId, request.serviceId);
    EXPECT_EQ(response.callId, request.callId);
    EXPECT_EQ(response.encoding, request.encoding);
    EXPECT_EQ(response.data, expectedSerializedResponse);
  }
}

TEST(FetchAssetTest, fetchExistingAsset) {
  auto wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
  EXPECT_EQ(std::future_status::ready, wsClient->connect(URI).wait_for(DEFAULT_TIMEOUT));

  const auto millisSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  const auto tmpFilePath =
    boost::filesystem::temp_directory_path() / std::to_string(millisSinceEpoch.count());
  constexpr char content[] = "Hello, world";
  FILE* tmpAssetFile = std::fopen(tmpFilePath.c_str(), "w");
  std::fputs(content, tmpAssetFile);
  std::fclose(tmpAssetFile);

  const std::string uri = std::string("file://") + tmpFilePath.string();
  const uint32_t requestId = 123;

  auto future = foxglove::waitForFetchAssetResponse(wsClient);
  wsClient->fetchAsset(uri, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  const foxglove::FetchAssetResponse response = future.get();

  EXPECT_EQ(response.requestId, requestId);
  EXPECT_EQ(response.status, foxglove::FetchAssetStatus::Success);
  // +1 since NULL terminator is not written to file.
  ASSERT_EQ(response.data.size() + 1ul, sizeof(content));
  EXPECT_EQ(0, std::memcmp(content, response.data.data(), response.data.size()));
  std::remove(tmpFilePath.c_str());
}

TEST(FetchAssetTest, fetchNonExistingAsset) {
  auto wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
  EXPECT_EQ(std::future_status::ready, wsClient->connect(URI).wait_for(DEFAULT_TIMEOUT));

  const std::string assetId = "file:///foo/bar";
  const uint32_t requestId = 456;

  auto future = foxglove::waitForFetchAssetResponse(wsClient);
  wsClient->fetchAsset(assetId, requestId);
  ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
  const foxglove::FetchAssetResponse response = future.get();

  EXPECT_EQ(response.requestId, requestId);
  EXPECT_EQ(response.status, foxglove::FetchAssetStatus::Error);
  EXPECT_FALSE(response.errorMessage.empty());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  // Give the server some time to start
  std::this_thread::sleep_for(std::chrono::seconds(2));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  const auto testResult = RUN_ALL_TESTS();
  spinner.stop();

  return testResult;
}
