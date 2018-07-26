#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys
import bitstring
import binascii
import select
import datetime
from sensor_msgs.msg import Range
from Socket import Socket
import serial
import geometry_msgs.msg
import tf
import tf2_ros
from PyQt4.Qt import QObject
from dynamic_reconfigure.server import Server
from stams_gui.cfg import WinchConfig


class SonarNotFound(Exception):
    """Firing mechanism port could not be found."""
    pass

class SonarNotConfigured(Exception):
    """Firing mechanism is not configured for communications."""
    pass

class TimeoutError(Exception):
    """Communication timed out."""
    pass



class DepthInfo(QObject):
    """
    *DepthInfo* class for firing system driver
    """
    def __init__(self, port="/dev/ttyUSB0", baudrate = 38400):
        """

        :param port:
        :param baudrate: Baud rate, 115200 by default (can be 9600-115200)
        """
        self.port = port
        self.baudrate = baudrate

        self.conn = None
        self.initialized = False
        self.configured = False

        # Create TransformStamped message
        self.transf = geometry_msgs.msg.TransformStamped()
        self.transf.header.frame_id = 'world'
        self.transf.child_frame_id = 'sonar'
        # Initialize values to publish
        self.transf.transform.translation.x = 0
        self.transf.transform.translation.y = 0
        self.transf.transform.translation.z = 0
        self.transf.transform.rotation.x = 0.0
        self.transf.transform.rotation.y = 0.0
        self.transf.transform.rotation.z = 0.0
        self.transf.transform.rotation.w = 1.0
        self.tfbroad = tf2_ros.TransformBroadcaster()

    def __enter__(self):
        """
        Initializes for first use
        """
        self.open()
        srv = Server(WinchConfig, self.config_callback)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Cleans up
        :param exc_type:
        :param exc_val:
        :param exc_tb:
        :return:
        """
        self.close()
        rospy.loginfo("Closed depth reader on %s", self.port)

    def open(self):
        """
        Initializes connection
        :return:
        """
        # Initialize the port
        if not self.conn:
            try:
                self.conn = Socket(port = self.port, baudrate = self.baudrate)
                self.conn.conn.open()
            except OSError as e:
                raise SonarNotFound(self.port,e)

        rospy.loginfo("Initializing connection with depth board on %s", self.port)
        self.initialized = True
        #self.read()


    def close(self):
        self.conn.close()


    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure request: {winch_port_baudrate}, {winch_port}""".format(**config))
        self.set_params(**config)
        return config

    def set_params(self, winch_port_baudrate = None,winch_port = None, groups = None):
        self.port = winch_port
        self.baudrate = winch_port_baudrate
        self.close()
        self.conn = None
        self.open()
        #self.read()
        return self

    def send(self, message = None):
        self.conn.send(message)
        rospy.logdebug('%s sent', message)


    def read(self):
        """
        Receives information form port
        :return:
        """
        # send here something to verify sonar is connected?
        if not self.initialized:
            raise SonarNotConfigured(self.initialized)
        # Timeout count
        timeout_count = 0
        MAX_TIMEOUT_COUNT = 5

        # Scan until stopped
        self.preempted = False
        while not self.preempted:
            # Preempt on ROS shutdown
            if rospy.is_shutdown():
                self.preempt()
                return

            # Get the scan data
            try:
                data = self.get(wait = 1)
                self.transf.header.stamp = rospy.Time.now()
                self.transf.transform.translation.z = data
                self.tfbroad.sendTransform(self.transf)
                timeout_count = 0
            except TimeoutError:
                timeout_count += 1
                rospy.logdebug("Timeout count: %d", timeout_count)
                if timeout_count >= MAX_TIMEOUT_COUNT:
                    timeout_count = 0
                rospy.sleep(0.5)
                # Try again
                continue


    def get(self, wait = 2):
        """
        Sends command and returns reply
        :param message: Message to expect
        :param wait: Seconds to wait until received
        :return:
        """
        # Verify if sonar is initialized
        if not self.initialized:
            raise SonarNotConfigured

        #rospy.logdebug("Waiting for depth message")

        # Determine end time
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)

        # Wait until received
        while  datetime.datetime.now() < end:
            try:
                reply = self.conn.conn.read(4)

                inhex = int(reply.encode('hex'), 32)

                return inhex
            except:
                break

        # Timeout
        rospy.logerr("Timed out before receiving depth message")
        raise TimeoutError()

    def preempt(self):
        """
        Preempts the process
        :return:
        """
        rospy.logwarn("Preempting depth reader process...")
        self.preempted = True


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('depth_driver', log_level=rospy.DEBUG)

    port = '/dev/ttyUSB0'
    baudrate = 38400

    with DepthInfo(port,baudrate) as winch:
        winch.read()
