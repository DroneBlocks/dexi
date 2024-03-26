#!/usr/bin/env python

# Copyright (C) 2020 Copter Express Technologies, 2024 DroneBlocks, LLC
#
# Author: Oleg Kalachev <okalachev@gmail.com>, Dennis Baldwin <db@droneblocks.io>
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

        self.stop_mission = None
        self.block = String()
        self.published_block = String()
        self.running_lock = threading.Lock()

        self.running_pub = self.create_publisher(Bool, '~/running', qos_profile_1)
        self.block_pub = self.create_publisher(String, '~/block', qos_profile_1)
        self.print_pub = self.create_publisher(String, '~/print', qos_profile_10)
        self.prompt_pub = self.create_publisher(Prompt, '~/prompt', qos_profile_10)
        self.error_pub = self.create_publisher(String, '~/error', qos_profile_10)

        self.is_mission_running = Bool() # False
        self.running_pub.publish(self.is_mission_running)

        # TODO: somehow get the program path working where the .. causes problems with colcon build --symlink-install --packages-select droneblocks
        # self.declare_parameter('~/programs_dir', os.path.dirname(os.path.abspath(__file__)) + '../programs')
        self.declare_parameter('~/programs_dir', '/root/ros2_ws/build/droneblocks/programs')
        self.programs_path = self.get_parameter('~/programs_dir').value
        self.name_regexp = re.compile(r'^[a-zA-Z-_.]{0,20}$')

    def run(self, request, response):

        self.get_logger().info('code: ' + request.code)

        if not self.running_lock.acquire(False):
            self.get_logger().info('Already running')
            response.message = 'Already running'

        try:
            self.is_mission_running.data = True
            self.running_pub.publish(self.is_mission_running)

            def program_thread():
                self.get_logger().info('Inside program thread')
                self.stop_mission = False

                try:
                    g = {'rclpy': rclpy,
                        '_b': self.change_block,
                        'print': self._print,
                        'raw_input': self._input}
                    exec(request.code, g)
                except Stop:
                    self.get_logger().info('Program stopped')
                except Exception as e:
                    self.get_logger().error(str(e))

                self.running_lock.release()
                self.is_mission_running.data = False
                self.running_pub.publish(self.is_mission_running)
                self.change_block('')

            t = threading.Thread(target=program_thread)
            t.start()

            response.success = True
            response.message = 'Testing'
            return response

        except Exception as e:
            response.success = True
            response.message = 'Error'
            return response
            
    def run_defunct(self, req):
        if not self.running_lock.acquire(False):
            return {'message': 'Already running'}

        try:
            self.get_logger().info('Run program')
            self.running_pub.publish(Bool(True))

            def program_thread():
                self.stop = False
                g = {'rclpy': rclpy,
                    '_b': self.change_block,
                    'print': self._print,
                    'raw_input': self._input}
                try:
                    exec(req.code, g)
                except Stop:
                    self.get_logger().info('Program forced to stop')
                except Exception as e:
                    self.get_logger().error(str(e))
                    traceback.print_exc()
                    etype, value, tb = sys.exc_info()
                    fmt = traceback.format_exception(etype, value, tb)
                    fmt.pop(1) # remove 'clover_blocks' file frame
                    exc_info = ''.join(fmt)
                    self.error_pub.publish(str(e) + '\n\n' + exc_info)

                self.get_logger().info('Program terminated')
                self.running_lock.release()
                self.running_pub.publish(False)
                self.change_block('')

            t = threading.Thread(target=program_thread)
            t.start()

            return {'success': True}

        except Exception as e:
            self.running_lock.release()
            return {'message': str(e)}

    def stop(self, request, response):
        self.get_logger().info('Stop mission processing')
        self.stop_mission = True
        response.success = True
        response.message = 'Stop triggered'
        return response

    def load(self, request, response):
        response.names = []
        response.programs = []
        response.success = True
        try:
            for currentpath, folders, files in os.walk(self.programs_path):
                for f in files:
                    if not f.endswith('.xml'):
                        continue
                    filename = os.path.join(currentpath, f)
                    response.names.append(os.path.relpath(filename, self.programs_path))
                    response.programs.append(open(filename, 'r').read())
            return response
        except Exception as e:
            self.get_logger().error(e)
            response.message = str(e)
            return response

    def store(self, request, response):
        if not self.name_regexp.match(request.name):
            response.message = 'Bad mission name'
            return response

        filename = os.path.abspath(os.path.join(self.programs_path, request.name))

        try:
            open(filename, 'w').write(request.program)
            response.success = True
            response.message = 'Stored to ' + filename
            return response
        except Exception as e:
            self.get_logger().error(e)
            # TODO: understand why the structure below maps to the Load service format and not the Store format
            response.message = str(e)
            return response

    def change_block(self, _block):
        self.block.data = _block

        if self.block.data != self.published_block.data:
            self.get_logger().info('publishing block: ' + self.block.data)
            self.block_pub.publish(self.block)

        self.published_block.data = self.block.data

        if self.stop_mission: raise Stop

    def _print(self, s):
        self.get_logger().info(str(s))
        print_str = String()
        print_str.data = s
        self.print_pub.publish(print_str)

    def _input(self, s):
        self.get_logger().info('Input with message %s', s)
        prompt_id = str(uuid.uuid4()).replace('-', '')
        self.prompt_pub.publish(message=str(s), id=prompt_id)
        # TODO: fix below
        return rospy.wait_for_message('~input/' + prompt_id, String, timeout=30).data


def main(args=None):

    rclpy.init(args=args)

    droneblocks = DroneBlocks()
    droneblocks.create_service(Run, '~/run', droneblocks.run)
    droneblocks.create_service(Trigger, '~/stop', droneblocks.stop)
    droneblocks.create_service(Load, '~/load', droneblocks.load)
    droneblocks.create_service(Store, '~/store', droneblocks.store)
    droneblocks.get_logger().info('DroneBlocks node is ready')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(droneblocks)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    frequency = droneblocks.create_rate(10) # 10 Hz

    try:
        while rclpy.ok():
            frequency.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()