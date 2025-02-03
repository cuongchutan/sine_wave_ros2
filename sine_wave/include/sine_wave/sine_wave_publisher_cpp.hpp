#ifndef SINE_WAVE_PUBLISHER_CPP_HPP
#define SINE_WAVE_PUBLISHER_CPP_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "grayscale_image/srv/convert_to_grayscale.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <memory>
#include "sine_wave/publisher_params_cpp.hpp"

namespace sine_wave {

class SineWavePublisher : public rclcpp::Node
{
public:
    SineWavePublisher();

    void publish_sine_wave();
    void grayscale_srv_callback(const std::shared_ptr<grayscale_image::srv::ConvertToGrayscale::Request> request,
                                std::shared_ptr<grayscale_image::srv::ConvertToGrayscale::Response> response);

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<grayscale_image::srv::ConvertToGrayscale>::SharedPtr srv;
    rclcpp::Time t0;

    double publisher_frequency;
    double amplitude;
    double angular_frequency;
    double phase;
    double t;

    std::shared_ptr<sine_wave::ParamListener> param_listener;
    sine_wave::Params params;
};

} // namespace sine_wave

#endif  // SINE_WAVE_PUBLISHER_CPP_HPP
