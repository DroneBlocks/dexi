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
        self.stop = None
        self.block = ''
        self.published_block = None
        self.running_lock = threading.Lock()

        self.running_pub = self.ceate_publisher('~running', Bool, queue_size=1, latch=True)
        self.block_pub = self.ceate_publisher('~block', String, queue_size=1, latch=True)
        print_pub = self.ceate_publisher('~print', String, queue_size=10)
        prompt_pub = self.ceate_publisher('~prompt', Prompt, queue_size=10)
        error_pub = self.ceate_publisher('~error', String, queue_size=10)
        self.running_pub.publish(False)

        self.published_block = None
        self.block = None

        rclpy.Timer(rclpy.Duration(rclpy.get_param('block_rate', 0.2)), publish_block)

    def publish_block(self, event):
        if self.published_block != self.block:
            self.block_pub.publish(self.block)
       
        self.published_block = self.block


    def change_block(self, _block):
        self. block = _block
        if stop: raise Stop


rospy_sleep = rospy.sleep


def sleep(duration):
    time_start = rospy.get_time()

    if isinstance(duration, rospy.Duration):
        duration = duration.to_sec()

    time_until = time_start + duration

    while not rospy.is_shutdown():
        if stop: raise Stop  # check stop condition every half-second
        if (time_until - rospy.get_time()) > 0.5:
            print('sleep 0.5')
            rospy_sleep(0.5)
        else:
            rospy_sleep(time_until - rospy.get_time())
            return


rospy.sleep = sleep
rospy.init_node = lambda *args, **kwargs: None


def _print(s):
    rospy.loginfo(str(s))
    print_pub.publish(str(s))


def _input(s):
    rospy.loginfo('Input with message %s', s)
    prompt_id = str(uuid.uuid4()).replace('-', '')
    prompt_pub.publish(message=str(s), id=prompt_id)
    return rospy.wait_for_message('~input/' + prompt_id, String, timeout=30).data;


def run(req):
    if not running_lock.acquire(False):
        return {'message': 'Already running'}

    try:
        rospy.loginfo('Run program')
        running_pub.publish(True)

        def program_thread():
            global stop
            stop = False
            g = {'rospy': rospy,
                '_b': change_block,
                'print': _print,
                'raw_input': _input}
            try:
                exec(req.code, g)
            except Stop:
                rospy.loginfo('Program forced to stop')
            except Exception as e:
                rospy.logerr(str(e))
                traceback.print_exc()
                etype, value, tb = sys.exc_info()
                fmt = traceback.format_exception(etype, value, tb)
                fmt.pop(1) # remove 'clover_blocks' file frame
                exc_info = ''.join(fmt)
                error_pub.publish(str(e) + '\n\n' + exc_info)

            rospy.loginfo('Program terminated')
            running_lock.release()
            running_pub.publish(False)
            change_block('')

        t = threading.Thread(target=program_thread)
        t.start()

        return {'success': True}

    except Exception as e:
        running_lock.release()
        return {'message': str(e)}


def stop(req):
    global stop
    rospy.loginfo('Stop program')
    stop = True
    return {'success': True}


# TODO: find dir in installed package
programs_path = rospy.get_param('~programs_dir', os.path.dirname(os.path.abspath(__file__)) + '/../programs')


def load(req):
    res = {'names': [], 'programs': [], 'success': True}
    try:
        for currentpath, folders, files in os.walk(programs_path):
            for f in files:
                if not f.endswith('.xml'):
                    continue
                filename = os.path.join(currentpath, f)
                res['names'].append(os.path.relpath(filename, programs_path))
                res['programs'].append(open(filename, 'r').read())
        return res
    except Exception as e:
        rospy.logerr(e)
        return {'names': [], 'programs': [], 'message': str(e)}


name_regexp = re.compile(r'^[a-zA-Z-_.]{0,20}$')

def store(req):
    if not name_regexp.match(req.name):
        return {'message': 'Bad program name'}

    filename = os.path.abspath(os.path.join(programs_path, req.name))
    try:
        open(filename, 'w').write(req.program)
        return {'success': True, 'message': 'Stored to ' + filename}
    except Exception as e:
        rospy.logerr(e)
        return {'names': [], 'programs': [], 'message': str(e)}


rospy.Service('~run', Run, run)
rospy.Service('~stop', Trigger, stop)
rospy.Service('~load', Load, load)
rospy.Service('~store', Store, store)


rospy.loginfo('Ready')
rospy.spin()
