#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os
from grayscale_image.srv import ConvertToGrayscale

pkg_install_dir = os.path.join(os.getenv('PWD'), 'install', 'sine_wave')
img_dir = os.path.join(pkg_install_dir, 'share', 'sine_wave', 'image')

class SineWaveSubscriber(Node):
    def __init__(self):
        super().__init__('sine_wave_subscriber')
        self.subscription = self.create_subscription(Float32, 'sine_wave', self.listener_callback, 10)

        # Add a client for the grayscale image service
        self.client = self.create_client(ConvertToGrayscale, 'grayscale_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available')
        self.request = ConvertToGrayscale.Request()

        self.get_logger().info('sine_wave_subscriber node is started.')


    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed: {msg.data}')

        # Usage of the client for the grayscale image service
        self.request.file_path = os.path.join(img_dir, 'demo_image.png')
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.service_callback)


    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call succeeded')
            self.get_logger().info(f'Grayscale image saved at: {response.grayscale_file_path}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
