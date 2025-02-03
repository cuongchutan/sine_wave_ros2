#include "sine_wave/sine_wave_publisher_cpp.hpp"

namespace sine_wave {

SineWavePublisher::SineWavePublisher()
    : Node("sine_wave_publisher_cpp")
{
    param_listener = std::make_shared<ParamListener>(get_node_parameters_interface());
    params = param_listener->get_params();

    publisher_frequency = params.publisher_frequency;
    amplitude = params.amplitude;
    angular_frequency = params.angular_frequency;
    phase = params.phase;

    // publisher_frequency = 100.0;
    // amplitude = 1.0;
    // angular_frequency = 1.0;
    // phase = 0.0;

    RCLCPP_INFO(this->get_logger(), "publisher_frequency: %f", publisher_frequency);
    RCLCPP_INFO(this->get_logger(), "amplitude: %f", amplitude);
    RCLCPP_INFO(this->get_logger(), "angular_frequency: %f", angular_frequency);
    RCLCPP_INFO(this->get_logger(), "phase: %f", phase);
    
    publisher = this->create_publisher<std_msgs::msg::Float32>("sine_wave_cpp", 10);
    timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publisher_frequency),
        std::bind(&SineWavePublisher::publish_sine_wave, this)
    );

    // Add a service to convert an image to grayscale
    srv = this->create_service<grayscale_image::srv::ConvertToGrayscale>(
        "grayscale_srv_cpp", std::bind(&SineWavePublisher::grayscale_srv_callback, 
        this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "sine_wave_publisher_cpp node is started.");
    t0 = this->now();
    t = 0.0;
}


void SineWavePublisher::publish_sine_wave()
{
    // sine_wave = A * sin(omega * t + phi_0)
    auto t_sys = this->now() - t0;
    t = t_sys.seconds();
    float sine_value = amplitude * std::sin(angular_frequency * t + phase);
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = sine_value;
    publisher->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published: %f", sine_value);
}


void SineWavePublisher::grayscale_srv_callback(
    const std::shared_ptr<grayscale_image::srv::ConvertToGrayscale::Request> request,
    std::shared_ptr<grayscale_image::srv::ConvertToGrayscale::Response> response)
{
    cv::Mat img = cv::imread(request->file_path, cv::IMREAD_COLOR);
    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read image from %s", request->file_path.c_str());
        return;
    }
    // Convert the image to grayscale
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_RGB2GRAY);
    // Save the converted image in the folder as the original image
    std::string img_dir = request->file_path.substr(0, request->file_path.find_last_of('/'));
    std::string grayscale_file_path = img_dir + "/grayscale_image_cpp.png";
    cv::imwrite(grayscale_file_path, gray_img);
    response->grayscale_file_path = grayscale_file_path;
    RCLCPP_INFO(this->get_logger(), "Grayscale image saved at: %s", response->grayscale_file_path.c_str());
}

} // namespace sine_wave


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sine_wave::SineWavePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
