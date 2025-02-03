#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
# import yaml
import os
import cv2
from grayscale_image.srv import ConvertToGrayscale
from sine_wave.generate_publisher_params import sine_wave
from sine_wave.sine_wave_publisher import SineWavePublisher
import unittest
from unittest.mock import MagicMock, patch
import random


class TestSineWavePublisher(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        self.node = SineWavePublisher()
        self.node.publisher_frequency = 100.0
        self.node.amplitude = 1.0
        self.node.angular_frequency = 1.0
        self.node.phase = 0.0
        self.node.t = random.uniform(0.0, 10.0)


    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


    def test_publish_sine_wave(self):
        # Prepare initializations
        self.node.publisher.publish = MagicMock()
        # Call the method that publishs the data
        self.node.publish_sine_wave()
        self.node.publisher.publish.assert_called_once()
        # Testing
        sine_value = self.node.publisher.publish.call_args[0][0].data
        expected_sine_value = self.node.amplitude * np.sin(self.node.angular_frequency * self.node.t + self.node.phase)
        self.assertEqual(sine_value, expected_sine_value)


if __name__ == '__main__':
    unittest.main()
