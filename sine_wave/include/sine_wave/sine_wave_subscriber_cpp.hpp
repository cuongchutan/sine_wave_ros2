#ifndef SINE_WAVE_SUBSCRIBER_HPP
#define SINE_WAVE_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "grayscale_image/srv/convert_to_grayscale.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <memory>

class SineWaveSubscriber : public rclcpp::Node
{
public:
    SineWaveSubscriber();

    void listener_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void service_callback(rclcpp::Client<grayscale_image::srv::ConvertToGrayscale>::SharedFuture future);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription;
    rclcpp::Client<grayscale_image::srv::ConvertToGrayscale>::SharedPtr client;
    std::shared_ptr<grayscale_image::srv::ConvertToGrayscale::Request> request;
};

#endif  // SINE_WAVE_SUBSCRIBER_HPP
