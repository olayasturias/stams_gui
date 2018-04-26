#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import bitstring
import binascii
import select
import datetime
from sensor_msgs.msg import Range
from Socket import Socket


class SonarNotFound(Exception):
    """Firing mechanism port could not be found."""
    pass

class SonarNotConfigured(Exception):
    """Firing mechanism is not configured for communications."""
    pass

class TimeoutError(Exception):
    """Communication timed out."""
    pass



class Firing_Sys(object):
    """
    *Firing_Sys* class for firing system driver
    """
    def __init__(self, port="/dev/ttyUSB0", baudrate = 115200):
        """

        :param port:
        :param baudrate: Baud rate, 115200 by default (can be 9600-115200)
        """
        self.port = port
        self.baudrate = baudrate

        self.conn = None
        self.initialized = False
        self.configured = False

        self.range = 0
        self.max_range = 0
        self.min_range = 0

    def __enter__(self):
        """
        Initializes for first use
        """
        self.open()
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
        rospy.loginfo("Closed firing mechanism on %s", self.port)

    def open(self):
        """
        Initializes sonar connection
        :return:
        """
        # Initialize the port
        if not self.conn:
            try:
                self.conn = Socket(self.port, self.baudrate)
            except OSError as e:
                raise SonarNotFound(self.port,e)

        rospy.loginfo("Initializing connection with firing mechanism on %s", self.port)
        self.initialized = True
        self.read()


    def close(self):
        self.conn.close()

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
                data = self.get('AFS',wait = 1).payload
                self.range = float(data)
                timeout_count = 0
            except TimeoutError:
                timeout_count += 1
                rospy.logdebug("Timeout count: %d", timeout_count)
                if timeout_count >= MAX_TIMEOUT_COUNT:
                    timeout_count = 0
                # Try again
                continue


    def get(self, message = None, wait = 2):
        """
        Sends command and returns reply
        :param message: Message to expect
        :param wait: Seconds to wait until received
        :return:
        """
        # Verify if sonar is initialized
        if not self.initialized:
            raise SonarNotConfigured

        rospy.logdebug("Waiting for limit switches state message")

        # Determine end time
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)

        # Wait until received 
        while  datetime.datetime.now() < end:
            try:
                reply = self.conn.get_reply(expected_reply = message, enabled = True)
                return reply
            except:
                break

        # Timeout
        rospy.logerr("Timed out before receiving limit switches message")
        raise TimeoutError()

    def preempt(self):
        """
        Preempts the process
        :return:
        """
        rospy.logwarn("Preempting fixing process...")
        self.preempted = True


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('firing_driver', log_level=rospy.DEBUG)

    port = '/dev/ttyUSB0'
    baudrate = 115200

    with Firing_Sys(port,baudrate) as firing_system:
        pass