#include <gtest/gtest.h>

#include <foxglove_bridge/serialization.hpp>

TEST(SerializationTest, ServiceRequestSerialization) {
  foxglove::ServiceRequest req;
  req.serviceId = 2;
  req.callId = 1;
  req.encoding = "json";
  req.data = {1, 2, 3};

  std::vector<uint8_t> data(req.size());
  req.write(data.data());

  foxglove::ServiceRequest req2;
  req2.read(data.data(), data.size());
  EXPECT_EQ(req.serviceId, req2.serviceId);
  EXPECT_EQ(req.callId, req2.callId);
  EXPECT_EQ(req.encoding, req2.encoding);
  EXPECT_EQ(req.data.size(), req2.data.size());
  EXPECT_EQ(req.data, req2.data);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
