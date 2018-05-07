#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import numpy as np
import scipy.ndimage as ndi

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
# For the thrusters publisher
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Range

import serial

import dynamic_reconfigure.client

# For subscribers
#import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils

# For winch depth info
from depthstamp import DepthInfo
import tf2_ros
import tf

"""
.. codeauthor:: Olaya Alvarez Tunon
: file rov_ui.py
"""

class ParameterServer_Params(QThread):
    """
    This class includes the variables that the sensors have available in the ROS Parameter Server so they can be
    dinamically reconfigured from the User Interface.

    It is required for this parameters to be in a separate class, because in case of the sensors not being initialized
    yet, the variable cannot be defined and the Interface would freeze. Having it in a separate class, we run the
    definition of the variable in a separate thread so it doesn't freeze the Interface.

    **Atributes**:

    .. data:: profiler_client

    .. data:: valeport_altimeter_client

    the client of the elements for the dynamic configuration of the Tritech Profiler's
    parameters.

    """
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.profiler_client = None
        self.valeport_altimeter_client = None
        self.bowtech_camera = None
        self.winch_depth_client = None


    def run(self):
        """
        The run method is a overun method of the QThread class, and starts when required in a separate thread.

        """
        self.winch_depth_client = dynamic_reconfigure.client.Client("/depth_driver")
        self.valeport_altimeter_client = dynamic_reconfigure.client.Client("/valeport_altimeter")
        self.profiler_client = dynamic_reconfigure.client.Client("/tritech_profiler")
        



class SDS_Params():
    """ The *SDS_params* class contains all the required elements to command the Serial Data Switch.
    The SDS allows to power on and off the Tritech Profiler, the Sonar Altimeter and the Bowtech Camera.
    It also switches the information channel between the Tritech Profiler and the Sonar altimeter, since
    they cannot both transmit at the same time.
    """
    def __init__(self):

        self.baudrate = 115200
        self.previous_baudrate = 115200

        self.profiler_channel  = '/dev/ttyUSB0'
        self.camera_channel    = '/dev/ttyUSB0'
        self.altimeter_channel = '/dev/ttyUSB0'

        self.profiler_pow_channel  = 'A'
        self.camera_pow_channel    = 'C'
        self.altimeter_pow_channel = 'B'

        self.profiler_pow_enabled   = 0
        self.camera_pow_enabled     = 0
        self.altimeter_pow_enabled  = 0

        self.profiler_data_channel  = '2'
        self.camera_data_channel    = '1'
        self.altimeter_data_channel = '1'

    def parse_params(self):
        """
        This function parses the string messages that need to be transmitted to the SDS board to accomplish the
        required tasks

        :return:
        """
        self.profiler_pow_message  = '+++' + str(self.profiler_pow_channel) + str(self.profiler_pow_enabled) +' ;'
        self.camera_pow_message    = '+++' + str(self.camera_pow_channel)   + str(self.camera_pow_enabled)   + ';'
        self.altimeter_pow_message = '+++' + str(self.altimeter_pow_channel)+ str(self.altimeter_pow_enabled)+ ';'

        self.profiler_data_message  = '+++' + str(self.profiler_data_channel) + ';'
        self.camera_data_message    = '+++' + str(self.camera_data_channel)   + ';'
        self.altimeter_data_message = '+++' + str(self.altimeter_data_channel)+ ';'

        self.baudrate_message       = '+++' + 'M' + str('C' if (self.baudrate == 115200) else self.baudrate/9600)+';'

    def send(self, port, message):
        """
        Send the message with the configuration parameters to the SDS board

        :param port: communication port in which the SDS is connected to
        :param message: command to send to the SDS board

        :return:
        """
        ser = serial.Serial(port,self.baudrate)
        print self.baudrate
        ser.write(message)
        print message
        ser.close()

    def change_baudrate(self):
        ser = serial.Serial(self.profiler_channel,self.previous_baudrate)
        ser.write(self.baudrate_message)
        print self.baudrate_message
        ser.close()

class image_converter(QObject):
  """ The *image_converter* class contains methods to subscribe to an image
  topic, convert it from sensor_msgs to cv image, and emit a signal to update
  the gui window.

  **Attributes**:

    .. data:: newcameraimage

        A pyqt signal that will be emitted every time a new frame arrives

    .. data:: image_sub

        ros topic image subscriber

    .. data:: cv_image

        variable that stores the cv imaged converted from sensor_msgs format

  """
  newcameraimage = pyqtSignal()

  def subscribe(self, signal, imgtopic):
      """ Subscribes to a camera topic

      :param signal: signal related to the camera topic
      :param imgtopic: string with camera topic you want to substribe to

      """
      self.image_sub = rospy.Subscriber(imgtopic,Image,
                                      self.callback, (signal))

  def callback(self,data,signal):
      """ Receives the Image topic message and converts it from sensor_msgs
      to cv image using cv_bridge.
      """
      bridge = CvBridge()
      try:
        # Read and convert data
        self.cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
        rotated = imutils.rotate_bound(self.cv_image,90)
        self.cv_image = rotated
        # Emit the signal associated
        signal.emit()
      except CvBridgeError as e:
        print(e)


class altimeter_subscriber(QObject):
    """
    This class subscribes to the topic that includes the data read by the Altimeter, and emits the signal that
    updates the value shown in the User Interface
    """
    newsonarrange = pyqtSignal()

    def subscribe(self, signal1):
        """
        This method defines the subscriber to the topic where the data from the Sonar Altimeter is published

        """
        " Subscribes to range topic "
        self.range = 0
        self.datastring = ''
        rospy.Subscriber("/range", Range, self.altimeter_callback, (signal1))

    def altimeter_callback(self,data,signal1):
        """
        This method emits the signal that updates the Sonar Altimeter value in the User Interface Screen.

        :param data: this variable stores the last measurement from the Sonar Altimeter
        :param signal1: this is a input argument, that allows to emit the signal that has been passed through this variable.

        """
        self.range = data.range
        self.datastring = "{:.2f}".format(self.range)
        signal1.emit()

class Joystick_thread(QThread):
    """ *Joystick_thread* class creates a thread apart that updates the joystick measurement
    every certain time
    
    **Attributes**:

      .. data:: signal

      signal that is emitted to call the slot which updates the velocity commands
      
      .. data:: mutex

      this mutex only controls the access of the joystick of the buttons
      to write over the ROV commands

      .. data:: is_locked

      this variable controls that the joystick is not writing over the ROV a the same time the
      buttons are being pressed
      
    """
    def __init__(self):
      ''' Initialize the Joystick thread. Here the signal and the mutex are initialized
      '''
      QtCore.QThread.__init__(self)
      self.signal = QtCore.SIGNAL("signal")
      self.mutex  = QtCore.QMutex(mode = QMutex.Recursive)
      self.is_locked = 0

    def run(self):
      ''' This function is executed every 0.1 secs, and the joystick signal is emitted if
      the joystick is activated
      '''
      while 1:
        time.sleep(0.1)
        if not self.is_locked:
          self.mutex.lock()
          # emit joystick signal each 0.1 secs
          self.emit(self.signal, "joystick thread")
          self.mutex.unlock()


class DepthSubscriber(QThread):
    """
    This method calls the class *DepthSubscriber*, which reads the serial port
    of the winch enconder to publish the PCAS depth
    """
    def __init__(self):
        QtCore.QThread.__init__(self)
        self.tflistener = tf.TransformListener()
        self.signal = QtCore.SIGNAL("depth signal")

    def run(self):
        # This function opens the serial port of the winch encoder and reads it
        # until shutdown
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tflistener.lookupTransform('/world', '/sonar', rospy.Time(0))
                self.wdepth = str(trans[2])
                self.emit(self.signal, "winch thread")
                rospy.sleep(0.5)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


class Window(QtGui.QWidget):
    """ *Window* class inherits from QtGui.QWidget class.
    In this class the UI for the different hardware devices is created

    **Attributes**:

      .. data:: lblgui

          lblgui is the text that labels the scene selection combo box

      .. data:: lblnav

          lblnav is the text that labels the Navigation mode combo box

      .. data:: lblnavpcas

          lblnavpcas is the text that labels the Navigation buttons for pcas

      .. data:: guimode

          is the combo box that chooses between simulator o real device

      .. data:: navmode

          navmode is the combo box that chooses between teleoperated or autonomous navigation

      .. data:: jb

          joystick button instance from pyqtgraph

      .. data:: jthread

          joystick thread instance

      .. data:: btnLEFT

          navigation button which moves the ROV to the left

      .. data:: btnLEFTpcas

           navigation button which rotates the RPIM anti-clockwise

      .. data:: btnRGT

          navigation button which moves the ROV to the right

      .. data:: btnRGTpcas

          navigation button which rotates the RPIM clockwise

      .. data:: btnUP

          navigation button which moves the ROV up

      .. data:: btnUPpcas

          navigation button which moves the RPIM up

      .. data:: btnDOWN

          navigation button which moves the ROV down

      .. data:: btnDOWNpcas

          navigation button which moves the RPIM down

      .. data:: imgwin

          instance of GraphicsLayoutWidget from pyqtgraph to show images

      .. data:: win

          instance of GLViewWidget where a GLScatterPlotItem will be added to show the trajectory

      .. data:: pcas

          instance of MyGLView where a GLScatterPlotItem will be added to show point cloud obtained by pcas

    """
    def __init__(self):
        # The super method returns the parent object of the Windw class, and we call its constructor
        super(Window, self).__init__()
        # Creation of the gui
        self.initUI()


    def initUI(self):
      ''' *initUI* method initializes the widgets from the UI window, and connects the different signals
      with the slots.
      '''
      # Global variable for the 3D graphics
      global win,curve

      # Class with Profiler/Camera and Altimeter parameters
      self.SDS_params = SDS_Params()

      ## TOOLTIPS ##

      # Font used to render tooltips, and text displayed
      QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
      self.setToolTip('This is a <b>QWidget</b> widget')

      ## LABELS ##

      # For ROV

      lblfix     = QtGui.QLabel('Installation tool', self)
      lblfixport = QtGui.QLabel('Port', self)
      lblfixshot = QtGui.QLabel('Trigger', self)

      linefix = QtGui.QFrame() # linea separatoria
      linefix.setFrameShape(QtGui.QFrame.HLine)
      linefix.setFrameShadow(QtGui.QFrame.Sunken)

      lblgui     = QtGui.QLabel('Scene selection', self)

      linescene = QtGui.QFrame() # linea separatoria
      linescene.setFrameShape(QtGui.QFrame.HLine)
      linescene.setFrameShadow(QtGui.QFrame.Sunken)

      lblnav     = QtGui.QLabel('Navigation Mode', self)

      linenav = QtGui.QFrame() # linea separatoria
      linenav.setFrameShape(QtGui.QFrame.HLine)
      linenav.setFrameShadow(QtGui.QFrame.Sunken)

      # For PCAS
      lbldepth = QtGui.QLabel('Depth')
      lblobstacle = QtGui.QLabel('Distance to obstacle')

      lblbaudrate               = QtGui.QLabel('Baudrate')

      linebr = QtGui.QFrame() # linea separatoria
      linebr.setFrameShape(QtGui.QFrame.HLine)
      linebr.setFrameShadow(QtGui.QFrame.Sunken)

      lblnavpcas                = QtGui.QLabel('Speed & Direction control', self)
      lblprofiling_sonar        = QtGui.QLabel('Profiling Sonar', self)
      lblprofiler_port_pcas     = QtGui.QLabel('Port', self)
      lblprofiler_pow_channel   = QtGui.QLabel('Power Supply Channel', self)
      lblprofiler_data_channel  = QtGui.QLabel('Data Channel', self)

      lineprofcoll = QtGui.QFrame() # linea separatoria
      lineprofcoll.setFrameShape(QtGui.QFrame.HLine)
      lineprofcoll.setFrameShadow(QtGui.QFrame.Sunken)

      lblcoll_camera            = QtGui.QLabel('Collision Camera', self)
      lblcam_pow_channel        = QtGui.QLabel('Power Supply Channel', self)
      lblcollcamera_port_pcas   = QtGui.QLabel('Port', self)

      linecollson = QtGui.QFrame() # linea separatoria
      linecollson.setFrameShape(QtGui.QFrame.HLine)
      linecollson.setFrameShadow(QtGui.QFrame.Sunken)

      lblsonar_alt              = QtGui.QLabel('Sonar Altimeter')
      lblsonaralt_port_pcas     = QtGui.QLabel('Port', self)
      lblalt_pow_channel        = QtGui.QLabel('Power Supply Channel', self)
      lblalt_data_channel       = QtGui.QLabel('Data Channel', self)

      linesonbut = QtGui.QFrame() # linea separatoria
      linesonbut.setFrameShape(QtGui.QFrame.HLine)
      linesonbut.setFrameShadow(QtGui.QFrame.Sunken)

      lblwinchdepth      = QtGui.QLabel('Winch Depth Info', self)
      lblwinchdepth_port = QtGui.QLabel('Port', self)

      linewinchdepth = QtGui.QFrame() # linea separatoria
      linewinchdepth.setFrameShape(QtGui.QFrame.HLine)
      linewinchdepth.setFrameShadow(QtGui.QFrame.Sunken)

      ## COMBO BOX ##

      # ROV

      combofix = QtGui.QComboBox(self)
      combofix.setEditable(True)


      # connect to funcions
      combofix.activated[str].connect(self.ComboFix_Port_Activated)

      guimode = QtGui.QComboBox(self)
      guimode.addItem("Simulator")
      guimode.addItem("Real ROV")

      guimode.activated[str].connect(self.Comboguimodeactivated)

      navmode = QtGui.QComboBox(self)
      navmode.addItem("Teleoperated")
      navmode.addItem("Autonomous")

      # PCAS

      # baudrate (sent only to SDS)

      comboBaudrate = QtGui.QComboBox(self)
      comboBaudrate.addItem("115200")
      comboBaudrate.addItem("57600")
      comboBaudrate.addItem("38400")
      comboBaudrate.addItem("28800")
      comboBaudrate.addItem("19200")
      comboBaudrate.addItem("14400")
      comboBaudrate.addItem("9600")

      comboBaudrate.activated[str].connect(self.ComboBaudActivated)

      # select serial port

      # for profiler
      comboprofiler_port_pcas = QtGui.QComboBox(self)
      comboprofiler_port_pcas.setEditable(True)

      comboprofiler_pow_channel = QtGui.QComboBox(self)
      comboprofiler_pow_channel.addItem("A")
      comboprofiler_pow_channel.addItem("B")
      comboprofiler_pow_channel.addItem("C")

      # connect to funcions
      comboprofiler_port_pcas.activated[str].connect(self.ComboProfiler_Port_Activated)
      comboprofiler_pow_channel.activated[str].connect(self.ComboProfiler_Pow_Activated)

      comboprofiler_data_channel = QtGui.QComboBox(self)
      comboprofiler_data_channel.addItem("2")
      comboprofiler_data_channel.addItem("1")

      # connect to funcions
      comboprofiler_data_channel.activated[str].connect(self.ComboProfiler_Data_Activated)

      # for camera
      combocollcamera_port_pcas = QtGui.QComboBox(self)
      combocollcamera_port_pcas.setEditable(True)

      combocam_pow_channel = QtGui.QComboBox(self)
      combocam_pow_channel.addItem("C")
      combocam_pow_channel.addItem("A")
      combocam_pow_channel.addItem("B")


      # connect to funcions
      combocollcamera_port_pcas.activated[str].connect(self.ComboCamera_Port_Activated)
      combocam_pow_channel.activated[str].connect(self.ComboCam_Pow_Activated)


      # for altimeter
      combosonaralt_port_pcas = QtGui.QComboBox(self)
      combosonaralt_port_pcas.setEditable(True)

      # For winch depth board
      combowinchdepth = QtGui.QComboBox(self)
      combowinchdepth.setEditable(True)

      # Add labels to combo box options
      for i in range (0, 4):
          comboprofiler_port_pcas.addItem(  "/dev/ttyUSB" + str(i))
          combocollcamera_port_pcas.addItem("/dev/ttyUSB" + str(i))
          combosonaralt_port_pcas.addItem(  "/dev/ttyUSB" + str(i))
          combofix.addItem(  "/dev/ttyUSB" + str(i))
          combowinchdepth.addItem("/dev/ttyUSB" + str(i))


      comboalt_pow_channel = QtGui.QComboBox(self)
      comboalt_pow_channel.addItem("B")
      comboalt_pow_channel.addItem("A")
      comboalt_pow_channel.addItem("C")

      # connect to funcions
      combosonaralt_port_pcas.activated[str].connect(self.ComboAltimeter_Port_Activated)
      comboalt_pow_channel.activated[str].connect(self.ComboAltimeter_Pow_Activated)
      combowinchdepth.activated[str].connect(self.ComboWinch_Port_Activated)

      comboalt_data_channel = QtGui.QComboBox(self)
      comboalt_data_channel.addItem("1")
      comboalt_data_channel.addItem("2")

      # connect to funcions
      comboalt_data_channel.activated[str].connect(self.ComboAltimeter_Data_Activated)

      ## CHECKBOX ##

      # PCAS

      # For power transmission selection

      self.cb_profiler_power = QtGui.QCheckBox('Power Up', self)
      self.cb_profiler_power.setChecked(True)
      self.cb_cam_power      = QtGui.QCheckBox('Power Up', self)
      self.cb_cam_power.setChecked(True)
      self.cb_alt_power      = QtGui.QCheckBox('Power Up', self)
      self.cb_alt_power.setChecked(True)

      # Callbacks

      self.cb_profiler_power.stateChanged.connect(self.ProfilerPowerCheckbox)
      self.cb_cam_power.stateChanged.connect(self.CameraPowerCheckbox)
      self.cb_alt_power.stateChanged.connect(self.AltimeterPowerCheckbox)

      # For data transmission selection

      self.cb_profiler_data = QtGui.QCheckBox('Data Transmission', self)
      #cb_cam_data      = QtGui.QCheckBox('Data Transmission', self)
      self.cb_alt_data      = QtGui.QCheckBox('Data Transmission', self)

      # Callbacks

      self.cb_profiler_data.stateChanged.connect(self.ProfilerDataCheckbox)
      #cb_cam_data.stateChanged.connect(self.CameraDataCheckbox)
      self.cb_alt_data.stateChanged.connect(self.AltimeterDataCheckbox)



      ## JOYSTICK ##
      self.jb = pg.JoystickButton()

      self.jb.setFixedWidth(120)
      self.jb.setFixedHeight(120)
      region = QtGui.QRegion(QtCore.QRect(self.jb.x()+5, self.jb.y()+5 ,110 ,110), QtGui.QRegion.Ellipse)
      self.jb.setStyleSheet("background-color: rgb(176,196,222);\n"
                            "border:1px solid rgb(0,0,0);")
      self.jb.setMask(region)

      self.jthread = Joystick_thread()
      self.connect(self.jthread, self.jthread.signal, self.update_joystick)

      self.jb.sigStateChanged.connect(self.joystick_changed)

      self.jthread.start()

      ## BUTTONS ##

      # for ROV

      # Buttons for fixing mechanism activation
      btnfix1 = QtGui.QPushButton('1',self)
      self.connect(btnfix1, QtCore.SIGNAL("clicked()"),
                   self.btnfix1_clicked)
      btnfix2 = QtGui.QPushButton('2',self)
      self.connect(btnfix2, QtCore.SIGNAL("clicked()"),
                   self.btnfix2_clicked)
      btnfix3 = QtGui.QPushButton('3',self)
      self.connect(btnfix3, QtCore.SIGNAL("clicked()"),
                   self.btnfix3_clicked)

      # Create button and set tooltip
      btnLEFT = QtGui.QPushButton('<', self)
      btnLEFT.setToolTip('Button for rotating anti-clockwise the ROV')
      # Moving and resizing button. sizehint gives recommended size
      btnLEFT.resize(btnLEFT.sizeHint())
      # Maintain it pressed when activated
      btnLEFT.setCheckable(True)
      # Keep executing function while button pressed
      btnLEFT.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnLEFT, QtCore.SIGNAL("clicked()"),
                   self.btnLEFT_clicked)

      # Create button and set tooltip
      btnRGT = QtGui.QPushButton('>', self)
      btnRGT.setToolTip('Button for rotating clockwise the ROV')
      # Moving and resizing button. sizehint gives recommended size
      btnRGT.resize(btnRGT.sizeHint())
      # Maintain it pressed when activated
      btnRGT.setCheckable(True)
      # Keep executing function while button pressed
      btnRGT.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnRGT, QtCore.SIGNAL("clicked()"),
                   self.btnRGT_clicked)

      # Create button and set tooltip
      btnUP = QtGui.QPushButton('UP', self)
      btnUP.setToolTip('Button for moving UP the ROV')
      # Moving and resizing button. sizehint gives recommended size
      btnUP.resize(btnUP.sizeHint())
      # Maintain it pressed when activated
      btnUP.setCheckable(True)
      # Keep executing function while button pressed
      btnUP.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnUP, QtCore.SIGNAL("clicked()"),
                   self.btnUP_clicked)

      # Create button and set tooltip
      btnDOWN = QtGui.QPushButton('DOWN', self)
      btnDOWN.setToolTip('Button for moving DOWN the ROV')
      # Moving and resizing button. sizehint gives recommended size
      btnDOWN.resize(btnDOWN.sizeHint())
      # Maintain it pressed when activated
      btnDOWN.setCheckable(True)
      # Keep executing function while button pressed
      btnDOWN.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnDOWN, QtCore.SIGNAL("clicked()"),
                   self.btnDOWN_clicked)

      # For PCAS

      # Create button and set tooltip
      btnUPpcas = QtGui.QPushButton('UP', self)
      btnUPpcas.setToolTip('Button for moving UP the RPIM')
      # Moving and resizing button. sizehint gives recommended size
      btnUPpcas.resize(btnUPpcas.sizeHint())
      # Keep executing function while button pressed
      btnUPpcas.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnUPpcas, QtCore.SIGNAL("clicked()"),
                   self.btnUPpcas_clicked)

      # Create button and set tooltip
      btnDOWNpcas = QtGui.QPushButton('DOWN', self)
      btnDOWNpcas.setToolTip('Button for moving DOWN the RPIM')
      # Moving and resizing button. sizehint gives recommended size
      btnDOWNpcas.resize(btnDOWNpcas.sizeHint())
      # Keep executing function while button pressed
      btnDOWNpcas.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnDOWNpcas, QtCore.SIGNAL("clicked()"),
                   self.btnDOWNpcas_clicked)

      btnLEFTpcas = QtGui.QPushButton('<', self)
      btnLEFTpcas.setToolTip('Button for rotating anti-clockwise the PCAS')
      # Moving and resizing button. sizehint gives recommended size
      btnLEFTpcas.resize(btnLEFTpcas.sizeHint())
      # Keep executing function while button pressed
      btnLEFTpcas.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnLEFTpcas, QtCore.SIGNAL("clicked()"),
                   self.btnLEFTpcas_clicked)

      # Create button and set tooltip
      btnRGTpcas = QtGui.QPushButton('>', self)
      btnRGTpcas.setToolTip('Button for rotating clockwise the PCAS')
      # Moving and resizing button. sizehint gives recommended size
      btnRGTpcas.resize(btnRGTpcas.sizeHint())
      # Keep executing function while button pressed
      btnRGTpcas.setAutoRepeat(True)
      # Connect button to slot
      self.connect(btnRGTpcas, QtCore.SIGNAL("clicked()"),
                   self.btnRGTpcas_clicked)

      # Create button and set tooltip
      btnSTOPpcas = QtGui.QPushButton('STOP', self)
      btnSTOPpcas.setToolTip('Button for STOPPING the PCAS')
      # Moving and resizing button. sizehint gives recommended size
      btnSTOPpcas.resize(btnSTOPpcas.sizeHint())
      # Keep executing function while button pressed
      btnSTOPpcas.setAutoRepeat(True)

      btnSTOPpcas.setStyleSheet('QPushButton {background-color: #E20202; color: white;}')
      btnSTOPpcas.setFixedSize(110,50)
      btnSTOPpcas.font().setBold(0)




      ## PLAIN TEXT ##

      # For ROV tab

      # Create plain text
      self.StatusText = QtGui.QPlainTextEdit("Waiting for first message")
      # With this, the user wont be able to edit plain text
      self.StatusText.setReadOnly(1)
      # Scroll automatically to the bottom of the text
      self.StatusText.centerCursor()

      # For PCAS tab

      # Collition avoidance text
      self.StatusText2 = QtGui.QPlainTextEdit("Waiting for first message")
      # With this, the user wont be able to edit plain text
      self.StatusText2.setReadOnly(1)
      # Scroll automatically to the bottom of the text
      self.StatusText2.centerCursor()
      #Depth text
      self.DepthText = QtGui.QPlainTextEdit("Waiting for first message")
      # With this, the user wont be able to edit plain text
      self.DepthText.setReadOnly(1)
      # Scroll automatically to the bottom of the text
      self.DepthText.centerCursor()
      self.DepthText.setFixedHeight(30)

      ## IMAGE VIEW ##

      # Create window with GraphicsView widget
      self.imgwin = pg.GraphicsLayoutWidget()

      view = self.imgwin.addViewBox()
      # Create image item
      self.img = pg.ImageItem(border='w')
      #print self.img.dataTransform()
      view.addItem(self.img)
      self.ic = image_converter()
      self.imgwin.rotate(90.0)

      # For PCAS
      self.imgwinpcas = pg.GraphicsLayoutWidget()
      self.imgwinpcas.rotate(90.0)
      viewpcas = self.imgwinpcas.addViewBox()
      # Create image item
      self.collisioncameraimg = pg.ImageItem(border='w')
      viewpcas.addItem(self.collisioncameraimg)
      self.collision_ic = image_converter()

      ## 3D GRAPICS ##


      # For sonar altimeter
      self.altimeter = altimeter_subscriber()


      ## TABS ##
      tab_widget = QtGui.QTabWidget()
      tab1       = QtGui.QWidget()
      tab2       = QtGui.QWidget()

      ## POSITIONING ##

      # create layout with ROV image and collision altimeter text
      layoutImgStat = QtGui.QVBoxLayout()
      layoutImgStat.addWidget(self.imgwin)
      layoutImgStat.addWidget(self.StatusText)
      # Put it inside a QWidget so it can be added to QSplitter
      layoutw = QtGui.QWidget()
      layoutw.setLayout(layoutImgStat)

      splitter = QtGui.QSplitter(self)
      splitter.addWidget(layoutw)
      #splitter.addWidget(self.StatusText)

      # 3D graphics and Status text vertical between them, form layoutV1
      layoutV1 = QtGui.QVBoxLayout()
      layoutV1.addWidget(splitter)

      # BtnUP and btnDOWN vertical between them (with joystick), form layoutV2
      layoutV2 = QtGui.QVBoxLayout()
      layoutV2.addWidget(btnUP)
      layoutV2.addWidget(self.jb)
      layoutV2.addWidget(btnDOWN)

      #BtnRIGHT, layoutV2 and btnLEFT placed horizontally, form LayoutH1
      layoutH1 = QtGui.QHBoxLayout()
      layoutH1.addWidget(btnLEFT)
      layoutH1.addLayout(layoutV2)
      layoutH1.addWidget(btnRGT)

      # Fixing mechanism labels
      layoutFix1 = QtGui.QVBoxLayout()
      layoutFix1.addWidget(lblfixport)
      layoutFix1.addWidget(lblfixshot)

      # FIXING buttons

      layoutFixBtn = QtGui.QHBoxLayout()
      layoutFixBtn.addWidget(btnfix1)
      layoutFixBtn.addWidget(btnfix2) 
      layoutFixBtn.addWidget(btnfix3) 

      # Fixing combo and buttons
      layoutFix2 = QtGui.QVBoxLayout()
      layoutFix2.addWidget(combofix)
      layoutFix2.addLayout(layoutFixBtn)

      # Fixing elements together
      layoutFixtot = QtGui.QHBoxLayout()
      layoutFixtot.addLayout(layoutFix1)
      layoutFixtot.addLayout(layoutFix2)

      # 4th vertical column: fix/scene sel./ navigation/ joystick
      layoutV3 = QtGui.QVBoxLayout()
      layoutV3.addWidget(lblfix)
      layoutV3.addLayout(layoutFixtot)
      layoutV3.addWidget(linefix)
      layoutV3.addWidget(lblgui)
      layoutV3.addWidget(guimode)
      layoutV3.addWidget(linescene)
      layoutV3.addWidget(lblnav)
      layoutV3.addWidget(navmode)
      layoutV3.addWidget(linenav)

      # Mode selection with navigation controllers
      layoutV4 = QtGui.QVBoxLayout()
      layoutV4.addLayout(layoutV3)
      layoutV4.addLayout(layoutH1)

      # LayoutV1 and LayoutV2 horizontal
      layoutH2 = QtGui.QHBoxLayout(tab1)
      layoutH2.addLayout(layoutV1)
      layoutH2.addLayout(layoutV4)

      # Layout of PCAS tab

      # winch nav buttons

      # BtnUP and btnDOWN vertical between them (with joystick), form layoutV2
      layout1V1 = QtGui.QVBoxLayout()
      layout1V1.addWidget(btnUPpcas)
      layout1V1.addWidget(btnSTOPpcas)
      layout1V1.addWidget(btnDOWNpcas)
      # BtnRIGHT, layoutV2 and btnLEFT placed horizontally, form LayoutH1
      layout1H1 = QtGui.QHBoxLayout()
      layout1H1.addWidget(btnLEFTpcas)
      layout1H1.addLayout(layout1V1)
      layout1H1.addWidget(btnRGTpcas)

      layout2 = QtGui.QVBoxLayout()
      layout2.addWidget(lblobstacle)
      layout2.addWidget(self.StatusText2)
      layout2.addWidget(lbldepth)
      layout2.addWidget(self.DepthText)
      layout2.addWidget(linesonbut)
      layout2.addWidget(lblnavpcas)
      layout2.addLayout(layout1H1)
      # Put it inside a QWidget so it can be added to QSplitter
      layoutw2 = QtGui.QWidget()
      layoutw2.setLayout(layout2)

      splitterpcas = QtGui.QSplitter(self)
      splitterpcas.addWidget(self.imgwinpcas)
      splitterpcas.addWidget(layoutw2)

      # Grouping sensor labels with their combo boxes

      layout1Hbaudrate = QtGui.QHBoxLayout()
      layout1Hbaudrate.addWidget(lblbaudrate)
      layout1Hbaudrate.addWidget(comboBaudrate)

      layout1Hport_profiler = QtGui.QHBoxLayout()
      layout1Hport_profiler.addWidget(lblprofiler_port_pcas)
      layout1Hport_profiler.addWidget(comboprofiler_port_pcas)

      layout1Hport_profiler2 = QtGui.QHBoxLayout()
      layout1Hport_profiler2.addWidget(lblprofiler_pow_channel)
      layout1Hport_profiler2.addWidget(comboprofiler_pow_channel)

      layout1Hport_profiler3 = QtGui.QHBoxLayout()
      layout1Hport_profiler3.addWidget(lblprofiler_data_channel)
      layout1Hport_profiler3.addWidget(comboprofiler_data_channel)

      layout1Hport_collcamera = QtGui.QHBoxLayout()
      layout1Hport_collcamera.addWidget(lblcollcamera_port_pcas)
      layout1Hport_collcamera.addWidget(combocollcamera_port_pcas)

      layout1Hcamera = QtGui.QHBoxLayout()
      layout1Hcamera.addWidget(lblcam_pow_channel)
      layout1Hcamera.addWidget(combocam_pow_channel)

      layout1Hport_sonaralt = QtGui.QHBoxLayout()
      layout1Hport_sonaralt.addWidget(lblsonaralt_port_pcas)
      layout1Hport_sonaralt.addWidget(combosonaralt_port_pcas)

      layout1Hport_sonaralt2 = QtGui.QHBoxLayout()
      layout1Hport_sonaralt2.addWidget(lblalt_pow_channel)
      layout1Hport_sonaralt2.addWidget(comboalt_pow_channel)

      layout1Hport_sonaralt3 = QtGui.QHBoxLayout()
      layout1Hport_sonaralt3.addWidget(lblalt_data_channel)
      layout1Hport_sonaralt3.addWidget(comboalt_data_channel)

      layoutwinch_port = QtGui.QHBoxLayout()
      layoutwinch_port.addWidget(lblwinchdepth_port)
      layoutwinch_port.addWidget(combowinchdepth)


      #label vertical, with nav buttons and combo boxes
      layout1V2 = QtGui.QVBoxLayout()
      layout1V2.addLayout(layout1Hbaudrate)
      layout1V2.addWidget(linebr)
      layout1V2.addWidget(lblprofiling_sonar)
      layout1V2.addLayout(layout1Hport_profiler)
      layout1V2.addLayout(layout1Hport_profiler2)
      layout1V2.addLayout(layout1Hport_profiler3)
      layout1V2.addWidget(self.cb_profiler_power)
      layout1V2.addWidget(self.cb_profiler_data)
      layout1V2.addWidget(lineprofcoll)
      layout1V2.addWidget(lblcoll_camera)
      layout1V2.addLayout(layout1Hport_collcamera)
      layout1V2.addLayout(layout1Hcamera)
      layout1V2.addWidget(self.cb_cam_power)
      layout1V2.addWidget(linecollson)
      layout1V2.addWidget(lblsonar_alt)
      layout1V2.addLayout(layout1Hport_sonaralt)
      layout1V2.addLayout(layout1Hport_sonaralt2)
      layout1V2.addLayout(layout1Hport_sonaralt3)
      layout1V2.addWidget(self.cb_alt_power)
      layout1V2.addWidget(self.cb_alt_data)
      layout1V2.addWidget(linewinchdepth)
      layout1V2.addWidget(lblwinchdepth)
      layout1V2.addLayout(layoutwinch_port)

      # buttons + scatter widget
      layout1H2 = QtGui.QHBoxLayout(tab2)
      layout1H2.addWidget(splitterpcas)
      layout1H2.addLayout(layout1V2)

      tab_widget.addTab(tab1, "ROV")
      tab_widget.addTab(tab2, "PCAS")

      mainLayout = QtGui.QVBoxLayout()
      mainLayout.addWidget(tab_widget)

      self.setLayout(mainLayout)

      ## WINDOW DATA ##

      self.setGeometry(300, 300, 250, 150)
      self.setWindowTitle('Reference Points Installation Module')
      self.setWindowIcon(QtGui.QIcon('stams.png'))

      ## TOPIC SUBSCRIBERS ##

      # Subscribe to pose message that needs to be displayed
      self.altimeter.subscribe(self.altimeter.newsonarrange)
      self.ic.subscribe(self.ic.newcameraimage,"/uwsim/camera1")
      self.collision_ic.subscribe(self.collision_ic.newcameraimage,"/v4l/bowtech_camera/image_raw")
      self.winchsubscriber = DepthSubscriber()
      self.winchsubscriber.start()

      ## SIGNALS ##

      # Create signal for the Plain text, connect to update_string function
      #self.ps.newstring.connect(self.update_string)
      # Signal for the new image received
      # for ROV
      self.ic.newcameraimage.connect(self.update_ROV_image)
      # for collision avoidance camera
      self.collision_ic.newcameraimage.connect(self.update_collisioncam_image)
      # For altimeter
      self.altimeter.newsonarrange.connect(self.update_string)
      # for depth board
      self.connect(self.winchsubscriber,self.winchsubscriber.signal,self.update_winch_depth)

      ## DISPLAY WIDGET ##
      # Display widget on screen
      self.show()

      ## PARAMETER SERVER ##
      self.ProfilerParam = ParameterServer_Params()
      self.ProfilerParam.start()
      #self.profiler_client = dynamic_reconfigure.client.Client("/tritech_profiler")
      self.showNormal()
    
    def update_winch_depth(self):
        #self.DepthText.appendPlainText(self.winchsubscriber.wdepth)
        self.DepthText.setPlainText(self.winchsubscriber.wdepth)

    def joystick_changed(self):
        """ This function unlocs the joystick thread, allowing it to update the
        joystick commands as the ROV thrusther's value
        """
        self.jthread.mutex.unlock()
        self.jthread.is_locked = 0

    def update_joystick(self):
        """ This function updates the joystick values and sends them to the ROV's
        thrusters
        """
        dx,dy = self.jb.getState()
        thrusters=[0,0,0,0,0]
        pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        thrusters[0]=thrusters[1]=dy
        thrusters[4]=-dx
        msg.data = thrusters
        pub.publish(msg)


    def update_string(self):
        ''' This function updates the text value fron the PlainText widget every time the
        value changes
        '''
        self.StatusText.appendPlainText('obstacle =' + self.altimeter.datastring)
        self.StatusText2.appendPlainText('obstacle =' + self.altimeter.datastring)


    def update_ROV_image(self):
        '''Every time a new image topic arrives, it is updated in the gui widget with this function
        '''
        self.img.setImage(self.ic.cv_image)
        #cv2.imshow("Image window", self.ic.cv_image)
        #cv2.waitKey(3)


    def update_collisioncam_image(self):
        '''Every time a new image topic arrives, it is updated in the gui widget with this function
        '''
        self.collisioncameraimg.setImage(self.collision_ic.cv_image)

    def btnfix1_clicked(self):
        print 'btn1'

    def btnfix2_clicked(self):
        print 'btn2'

    def btnfix3_clicked(self):
        print 'btn3'


    def btnLEFTpcas_clicked(self):
        #do stuff
        left = 1

    def btnLEFT_clicked(self):
        """ This function is executed when moving left button is clicked
        It publishes the required values in the thrusters to move left
      """
        self.jthread.mutex.tryLock()
        self.jthread.is_locked = 1
        thrusters=[0,0,0,0,0]
        pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        thrusters[0]=1
        thrusters[1]=-1
        msg.data = thrusters
        pub.publish(msg)

    def btnRGTpcas_clicked(self):
        #do stuff
        right = 1

    def btnRGT_clicked(self):
        """ This function is executed when moving right button is clicked
        It publishes the required values in the thrusters to move right
        """
        self.jthread.mutex.tryLock()
        self.jthread.is_locked = 1
        thrusters=[0,0,0,0,0]
        pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        thrusters[0]=-1
        thrusters[1]=1
        msg.data = thrusters
        pub.publish(msg)

    def btnUPpcas_clicked(self):
        #do stuff
        up = 1

    def btnUP_clicked(self):
        """ This function is executed when moving UP button is clicked
        It publishes the required values in the thrusters to move up
        """
        self.jthread.mutex.tryLock()
        self.jthread.is_locked = 1
        thrusters=[0,0,0,0,0]
        pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        thrusters[2] = thrusters[3]= 1
        msg.data = thrusters
        pub.publish(msg)

    def btnDOWNpcas_clicked(self):
        #do stuff
        down = 1

    def btnDOWN_clicked(self):
        """ This function is executed when moving down button is clicked
        It publishes the required values in the thrusters to move down
        """
        self.jthread.mutex.tryLock()
        self.jthread.is_locked = 1
        thrusters=[0,0,0,0,0]
        pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        thrusters[2] = thrusters[3]= -1
        msg.data = thrusters
        pub.publish(msg)

    def ComboFix_Port_Activated(self, txt):
        print txt

    def Comboguimodeactivated(self, modetxt):
        if modetxt == 'Real ROV':
            self.ic.subscribe(self.ic.newcameraimage,"/BlueRov2/image")
        else:
            self.ic.subscribe(self.ic.newcameraimage,"/uwsim/camera1")

    def ComboBaudActivated(self, baudtxt):
        """
        This method is called everytime the combo box with the Baudrate is activated.
        It sends the required command for configuring the baudrate in the Serial Data Switch
        :param baudtxt: the baudrate passed to a string value
        :return:
        """
        # Send commands to Serial Data Switch
        self.SDS_params.previous_baudrate = self.SDS_params.baudrate
        self.SDS_params.baudrate = int(baudtxt)

        self.SDS_params.parse_params()

        self.SDS_params.change_baudrate()

        # Reconfigure profiler and altimeter drivers
        profiler_params = {'profiler_port_baudrate': int(baudtxt)}
        altimeter_params = {'altimeter_port_baudrate': int(baudtxt)} 
        depth_params = {'winch_port_baudrate': int(baudtxt)}      
        
        
        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update profiler params. Are you sure Profiler is connected?")

        try:
            config = self.ProfilerParam.valeport_altimeter_client.update_configuration(altimeter_params)
        except:
            rospy.logwarn("Could not update Altimeter params. Are you sure Altimeter is connected?")

        try:
            config = self.ProfilerParam.winch_depth_client.update_configuration(depth_params)
        except:
            rospy.logwarn("Could not update depth board params. Are you sure Depth board is connected?")


    def ComboProfiler_Port_Activated(self,text):
        """
        This method is executed everytime the Combo Box for the Profiling Sonar communication port selection is
        activated.
        :param text: string value with the name of the port.
        :return:
        """
        self.SDS_params.profiler_channel = text

    def ComboCamera_Port_Activated(self,text):
        self.SDS_params.camera_channel = text

    def ComboAltimeter_Port_Activated(self,text):
        """
        This method is executed everytime the Combo Box for the Valeport Altimeter communication port selection is
        activated.
        :param text: string value with the name of the port.
        :return:
        """
        self.SDS_params.altimeter_channel = text

    def ComboProfiler_Pow_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.profiler_pow_channel = text

    def ComboProfiler_Data_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.profiler_data_channel = text

        profiler_params = {'profiler_port': self.SDS_params.profiler_data_channel}

        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update profiler params. Are you sure Profiler is connected?")


    def ComboCam_Pow_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.camera_pow_channel = text

    def ComboAltimeter_Pow_Activated(self,text):
        """This funcion is called when the Sonar altimeter combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Sonar altimeter is updated."""
        self.SDS_params.altimeter_pow_channel = text

    def ComboAltimeter_Data_Activated(self,text):
        """This funcion is called when the Sonar altimeter combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Sonar altimeter is updated."""
        self.SDS_params.altimeter_data_channel = text
    
    def ComboWinch_Port_Activated(self,text):
        
        depth_params = {'winch_port': text}      
        
        try:
            config = self.ProfilerParam.winch_depth_client.update_configuration(depth_params)
        except:
            rospy.logwarn("Could not update depth board params. Are you sure Depth board is connected?")


    def ProfilerPowerCheckbox(self,state):
        """
        Method that is executed everytime the state of the Checkbox for enabling/disabling Power supply changes.
        It involves the Power supply of the Profiling Sonar, and sends the required command to the Serial Data Switch
        :param state: valiable that stores the state of the checkbox
        :return:
        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.profiler_pow_enabled = 1
        else:
            self.SDS_params.profiler_pow_enabled = 0

        self.SDS_params.parse_params()
        self.SDS_params.send(self.SDS_params.profiler_channel, self.SDS_params.profiler_pow_message)

    def CameraPowerCheckbox(self,state):
        """
        Method that is executed everytime the state of the Checkbox for enabling/disabling Power supply changes.
        It involves the Power supply of the AntiCollision Camera, and sends the required command to the Serial Data Switch
        :param state: valiable that stores the state of the checkbox
        :return:

        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.camera_pow_enabled = 1
        else:
            self.SDS_params.camera_pow_enabled = 0

        self.SDS_params.parse_params()
        self.SDS_params.send(self.SDS_params.camera_channel, self.SDS_params.camera_pow_message)


    def AltimeterPowerCheckbox(self,state):
        """
        Method that is executed everytime the state of the Checkbox for enabling/disabling Power supply changes.
        It involves the Power supply of the Sonar Altimeter, and sends the required command to the Serial Data Switch
        :param state: valiable that stores the state of the checkbox
        :return:
        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.altimeter_pow_enabled = 1
        else:
            self.SDS_params.altimeter_pow_enabled = 0

        self.SDS_params.parse_params()
        self.SDS_params.send(self.SDS_params.altimeter_channel, self.SDS_params.altimeter_pow_message)

    def ProfilerDataCheckbox(self,state):
        if state == QtCore.Qt.Checked:
            self.SDS_params.profiler_data_enabled = 1
            self.cb_alt_data.setChecked(False)
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.profiler_channel, self.SDS_params.profiler_data_message)
            altimeter_params = {'altimeter_port_enabled': str(0)}
        else:
            self.SDS_params.profiler_data_enabled = 0

        profiler_params = {'port_enabled': str(self.SDS_params.profiler_data_enabled)}
        
        print profiler_params
        
        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update profiler params. Are you sure Profiler is connected?")

        try:
            config = self.ProfilerParam.valeport_altimeter_client.update_configuration(altimeter_params)
        except:
            rospy.logwarn("Could not update Altimeter params. Are you sure Altimeter is connected?")



    def AltimeterDataCheckbox(self,state):
        if state == QtCore.Qt.Checked:
            self.SDS_params.altimeter_data_enabled = 1
            self.cb_profiler_data.setChecked(False)
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.altimeter_channel, self.SDS_params.altimeter_data_message)
            profiler_params = {'port_enabled': str(0)}
        else:
            self.SDS_params.altimeter_data_enabled = 0

        altimeter_params = {'altimeter_port_enabled': str(self.SDS_params.altimeter_data_enabled)}


        try:
            config = self.ProfilerParam.valeport_altimeter_client.update_configuration(altimeter_params)
        except:
            rospy.logwarn("Could not update altimeter params. Are you sure Altimeter is connected?")

        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update params. Are you sure Profiler is connected?")
            





def main():
    rospy.init_node('guilistener', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
