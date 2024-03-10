#include <string>

#include <gtest/gtest.h>

#include <foxglove_bridge/foxglove_bridge.hpp>

TEST(VersionTest, TestWebSocketVersion) {
  // ex: "WebSocket++/0.8.1"
  const std::string version = foxglove::WebSocketUserAgent();
  EXPECT_EQ(version.substr(0, 14), "WebSocket++/0.");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
