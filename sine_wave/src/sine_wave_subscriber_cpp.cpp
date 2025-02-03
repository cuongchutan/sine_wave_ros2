#include "sine_wave/sine_wave_subscriber_cpp.hpp"

std::string pkg_install_dir = std::string(std::getenv("PWD")) + "/install/sine_wave";
std::string img_path = pkg_install_dir + "/share/sine_wave/image/demo_image.png";

SineWaveSubscriber::SineWaveSubscriber()
    : Node("sine_wave_subscriber_cpp")
{
    subscription = this->create_subscription<std_msgs::msg::Float32>(
        "sine_wave_cpp", 10,
        std::bind(&SineWaveSubscriber::listener_callback, this, std::placeholders::_1)
    );

    // Add a client for the grayscale image service
    client = this->create_client<grayscale_image::srv::ConvertToGrayscale>("grayscale_srv_cpp");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Service is not available");
    }
    request = std::make_shared<grayscale_image::srv::ConvertToGrayscale::Request>();

    RCLCPP_INFO(this->get_logger(), "sine_wave_subscriber_cpp node is started.");
}


void SineWaveSubscriber::listener_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Subscribed: %f", msg->data);

    // Usage of the client for the grayscale image service
    request->file_path = img_path;
    auto future = client->async_send_request(request);
    future.wait();
    this->service_callback(future.share());
}


void SineWaveSubscriber::service_callback(rclcpp::Client<grayscale_image::srv::ConvertToGrayscale>::SharedFuture future)
{
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
        RCLCPP_INFO(this->get_logger(), "Grayscale image saved at: %s", response->grayscale_file_path.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SineWaveSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
