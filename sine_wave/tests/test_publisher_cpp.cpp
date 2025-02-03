#include "../include/sine_wave/sine_wave_publisher_cpp.hpp"
#include <gtest/gtest.h>
#include <random>

class TestSineWavePublisher : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<sine_wave::SineWavePublisher>();
        node->publisher_frequency = 100.0;
        node->amplitude = 1.0;
        node->angular_frequency = 1.0;
        node->phase = 0.0;
        // Generate a random time
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 10.0);
        node->t = dis(gen);
    }


    void TearDown() override {
        node.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<sine_wave::SineWavePublisher> node;
};


TEST_F(TestSineWavePublisher, test_publish_sine_wave_cpp) {
    // Prepare initializations
    auto mock_publisher = std::make_shared<rclcpp::Publisher<std_msgs::msg::Float32>>();
    node->publisher = mock_publisher;
    // Call the method that publishs the data
    node->publish_sine_wave();
    // Testing
    auto sine_value = node->publisher->create_data->data;
    auto expected_sine_value = node->amplitude * std::sin(node->angular_frequency * node->t + node->phase);
    ASSERT_EQ(sine_value, expected_sine_value);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
