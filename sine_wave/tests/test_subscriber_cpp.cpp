#include "../include/sine_wave/sine_wave_subscriber_cpp.hpp"
#include <gtest/gtest.h>
#include <random>

class MockServiceClient {
public:
    MOCK_METHOD0(call, bool());
};


class MockLogger {
public:
    MOCK_METHOD1(info, void(const std::string &message));
};


class TestSineWaveSubscriber : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<SineWaveSubscriber>();

        // Mock necessary components to pass the "Service is not available" case
        mock_client = std::make_shared<MockServiceClient>();
        node->set_client(mock_client);

        mock_logger = std::make_shared<MockLogger>();
        node->set_logger(mock_logger);
    }


    void TearDown() override {
        node->destroy_node();
        rclcpp::shutdown();
    }

    std::shared_ptr<SineWaveSubscriber> node;
    std::shared_ptr<MockServiceClient> mock_client;
    std::shared_ptr<MockLogger> mock_logger;
};


TEST_F(TestSineWaveSubscriber, test_listener_callback_cpp) {
    // Prepare initializations
    auto msg = std::make_shared<std_msgs::msg::Float32>();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 10.0);
    msg->data = dis(gen);
    // Call the method that subscribes to the published data
    node->listener_callback(msg);
    // Testing
    EXPECT_CALL(*mock_logger, info("Subscribed: " + std::to_string(msg->data))).Times(1);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
