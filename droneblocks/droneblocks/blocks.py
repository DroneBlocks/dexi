#!/usr/bin/env python

# Copyright (C) 2020 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

from __future__ import print_function

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import os, sys
import traceback
import threading
import re
import uuid
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from dexi_msgs.msg import Prompt
from dexi_msgs.srv import Run, Load, Store


class Stop(Exception):
    pass

class DroneBlocks(Node):

    def __init__(self):
        super().__init__('droneblocks')

        qos_profile_1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_10 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        stop = None
        block = ''
        published_block = None
        running_lock = threading.Lock()

        running_pub = self.create_publisher(Bool, '~/running', qos_profile_1)
        block_pub = self.create_publisher(String, '~/block', qos_profile_1)
        print_pub = self.create_publisher(String, '~/print', qos_profile_10)
        prompt_pub = self.create_publisher(Prompt, '~/prompt', qos_profile_10)
        error_pub = self.create_publisher(String, '~/error', qos_profile_10)
        running_pub.publish(Bool()) # sends false

        #programs_path = self.get_parameter('~programs_dir', os.path.dirname(os.path.abspath(__file__)) + '/../programs')

        # rclpy.Timer(rclpy.Duration(self.get_parameter('block_rate', 0.2)), self.publish_block)

    def run(self, request, response):
        self.get_logger().info('code: ' + request.code)
        response.success = True
        response.message = 'Testing'
        return response

    def stop(self, request, response):
        self.get_logger().info('Stop mission processing')
        self.stop = True
        response.success = True
        response.message = 'Stop triggered'
        return response

    def load(self, request, response):
        print('load')

    def store(self, request, response):
        print('store')

    def publish_block(self, event):
        if self.published_block != self.block:
            self.block_pub.publish(self.block)
       
        self.published_block = self.block


    def change_block(self, _block):
        self. block = _block
        if self.stop: raise Stop


def main(args=None):

    rclpy.init(args=args)

    droneblocks = DroneBlocks()

    droneblocks.create_service(Run, '~/run', droneblocks.run)
    droneblocks.create_service(Trigger, '~/stop', droneblocks.stop)
    droneblocks.create_service(Load, '~/load', droneblocks.load)
    droneblocks.create_service(Store, '~/store', droneblocks.store)

    
    droneblocks.get_logger().info('Ready')
    rclpy.spin(droneblocks)
    rclpy.shutdown()

if __name__ == '__main__':
    main()