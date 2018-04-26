import rospy
import serial
import select
import bitstring
import Errors
from Replies import Reply
from Commands import Command


class Socket(object):
    """
    Sonar connection
    """
    def __init__(self, port,baudrate):
        """

        :param port:
        :param baudrate:
        """
        self.conn = serial.Serial(baudrate = baudrate,timeout=2)
        self.conn.port = port
        self.conn.baudrate = baudrate

    def open(self):
        """
        Opens serial connection
        :return:
        """
        self.conn.open()

    def close(self):
        """
        Closes serial connection
        :return:
        """
        self.conn.close()

    def send(self, message, payload = None, command = None):
        """
        :param message:
        :param payload:
        :return:
        """
        cmd = Command(message, payload,command)
        rospy.logdebug("Sending %s: %s", message, command)
        self.conn.write(cmd.serialize())

    def _readline(self):
        eol = b'*'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.conn.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    def get_reply(self, expected_reply = None, enabled = False):
        """
        Waits for and returns reply
        :return:
        """
        try:
            # Wait for the header character
            # Don't put anything in this while, because if losses packets if you do so
            if expected_reply:
                while not self.conn.read() == expected_reply:
                    pass

            # Initialize empty packet where the received stream will be saved
            packet = bitstring.BitStream()


            rospy.logdebug("Received valid packet with firing parameters")

            # Convert each caracter from received string stream in the bitstream
            current_line = []
            while enabled:
                try:
                    # Read the message sent by altimeter. It always has length 22
                    current_line.append(self.conn.readline())
                    break
                except:
                    continue


        except select.error as (code,msg):
             if code == errno.EINTR:
                 raise KeyboardInterrupt()
             raise

        rospy.logdebug("Received: %s", current_line)
        return current_line #reply