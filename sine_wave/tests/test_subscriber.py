# #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os
from grayscale_image.srv import ConvertToGrayscale
from sine_wave.sine_wave_subscriber import SineWaveSubscriber
import unittest
from unittest.mock import MagicMock, patch
import random


class TestSineWaveSubscriber(unittest.TestCase):

    @patch('sine_wave.sine_wave_subscriber.SineWaveSubscriber.create_client')
    @patch('sine_wave.sine_wave_subscriber.SineWaveSubscriber.create_subscription')
    @patch('grayscale_image.srv.ConvertToGrayscale')
    def setUp(self, mock_service, mock_create_subscription, mock_create_client):
        rclpy.init(args=None)
        self.node = SineWaveSubscriber()

        # Mock necessary components to pass the "Service is not available" case
        self.mock_service = mock_service
        self.mock_create_client = mock_create_client
        self.mock_create_subscription = mock_create_subscription

        self.mock_service_instance = mock_service.Request.return_value
        self.mock_client = MagicMock()
        self.node.client = self.mock_client
        self.node.request = self.mock_service_instance

        # Patch the logger
        self.logger_patcher = patch.object(self.node, 'get_logger')
        self.mock_logger = self.logger_patcher.start().return_value
        self.mock_logger.info = MagicMock()


    @patch('grayscale_image.srv.ConvertToGrayscale.Response')
    def test_listener_callback(self, mock_response):
        # Prepare initializations
        mock_msg = Float32()
        mock_msg.data = random.uniform(0.0, 10.0)
        # Call the method that subscribe the published data
        self.node.listener_callback(mock_msg)
        # Tesing
        self.mock_logger.info.assert_called_with(f'Subscribed: {mock_msg.data}')


    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
