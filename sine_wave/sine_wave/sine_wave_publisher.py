#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Float32
import numpy as np
import yaml
import os
import cv2
from grayscale_image.srv import ConvertToGrayscale
from sine_wave.generate_publisher_params import sine_wave

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')

        self.param_listener = sine_wave.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.publisher_frequency = self.params.publisher_frequency
        self.amplitude = self.params.amplitude
        self.angular_frequency = self.params.angular_frequency
        self.phase = self.params.phase

        # self.publisher_frequency = 100.0
        # self.amplitude = 1.0
        # self.angular_frequency = 1.0
        # self.phase = 0.0

        self.publisher = self.create_publisher(Float32, 'sine_wave', 10)
        self.timer = self.create_timer(1.0 / self.publisher_frequency, self.publish_sine_wave)

        # Add a service to convert an image to grayscale
        self.srv = self.create_service(ConvertToGrayscale, 'grayscale_srv', self.grayscale_srv_callback)

        self.get_logger().info('sine_wave_publisher node is started.')
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.t0 = self.clock.now()
        self.t = 0.0


    def publish_sine_wave(self):
        # sine_wave = A * sin(omega * t + phi_0)
        self.t = (self.clock.now() - self.t0).nanoseconds * 1e-9
        sine_value = self.amplitude * np.sin(self.angular_frequency * self.t + self.phase)
        msg = Float32()
        msg.data = sine_value
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {sine_value}')


    def grayscale_srv_callback(self, request, response):
        img = cv2.imread(request.file_path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error(f'Failed to read image from {request.file_path}')
            return response
        # Convert the image to grayscale
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # Save the converted image in the folder as the original image
        img_dir = os.path.dirname(request.file_path)
        grayscale_file_path = os.path.join(img_dir, 'grayscale_image.png')
        cv2.imwrite(grayscale_file_path, gray_img)
        response.grayscale_file_path = grayscale_file_path
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
