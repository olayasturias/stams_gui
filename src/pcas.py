#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent, QStyle
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

from pc2ply import SavePointCloud

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
        self.valeport_altimeter_client = dynamic_reconfigure.client.Client("/valeport_altimeter")
        self.profiler_client = dynamic_reconfigure.client.Client("/tritech_profiler")
        self.winch_depth_client = dynamic_reconfigure.client.Client("/depth_driver")





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

        # self.profiler_pow_channel  = 'A'
        # self.camera_pow_channel    = 'C'
        # self.altimeter_pow_channel = 'B'

        self.profiler_pow_enabled   = 0
        self.camera_pow_enabled     = 0
        self.altimeter_pow_enabled  = 0

        self.profiler_data_enabled   = 0
        self.camera_data_enabled   = 0
        self.altimeter_data_enabled  = 0

        # self.profiler_data_channel  = '2'
        # self.camera_data_channel    = '1'
        # self.altimeter_data_channel = '1'

    def parse_params(self):
        """
        This function parses the string messages that need to be transmitted to the SDS board to accomplish the
        required tasks

        :return:
        """
        self.profiler_pow_message  = '#0' + 'SON' + str(self.profiler_pow_enabled) + '\r'
        self.camera_pow_message    = '#0' + 'CAM' + str(self.camera_pow_enabled) + '\r'
        self.altimeter_pow_message = '#0' + 'ALT' + str(self.altimeter_pow_enabled) + '\r'

        self.profiler_data_message  = '#0SSO' + str(self.profiler_data_enabled) + '\r'
        self.camera_data_message    = '#0VID' + str(int(not self.camera_data_enabled)) + '\r'
        self.altimeter_data_message = '#0SSO' + str(int(not self.altimeter_data_enabled))+ '\r'

        #self.baudrate_message       = '#0' + 'M' + str('C' if (self.baudrate == 115200) else self.baudrate/9600) + '\r'

    def send(self, port, message):
        """
        Send the message with the configuration parameters to the SDS board

        :param port: communication port in which the SDS is connected to
        :param message: command to send to the SDS board

        :return:
        """
        ser = serial.Serial(port,self.baudrate)
        ser.write(message)
        print 'SERIAL MESSAGE SENT'
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

      .. data:: lblrecord

          lblrecord is the text that labels the Recording buttons for pcas

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


      ## LABELS ##


      # For PCAS
      lbldepth = QtGui.QLabel('Depth')
      lblobstacle = QtGui.QLabel('Distance to obstacle')


      linebr = QtGui.QFrame() # linea separatoria
      linebr.setFrameShape(QtGui.QFrame.HLine)
      linebr.setFrameShadow(QtGui.QFrame.Sunken)

      lblrecord                 = QtGui.QLabel('Record Point Cloud', self)
      lblprofiling_sonar        = QtGui.QLabel('Profiling Sonar', self)
      lblprofiler_port_pcas     = QtGui.QLabel('Port', self)


      lineprofcoll = QtGui.QFrame() # linea separatoria
      lineprofcoll.setFrameShape(QtGui.QFrame.HLine)
      lineprofcoll.setFrameShadow(QtGui.QFrame.Sunken)

      lblcoll_camera            = QtGui.QLabel('Collision Camera', self)
      # lblcam_pow_channel        = QtGui.QLabel('Power Supply Channel', self)
      lblcollcamera_port_pcas   = QtGui.QLabel('Port', self)

      linecollson = QtGui.QFrame() # linea separatoria
      linecollson.setFrameShape(QtGui.QFrame.HLine)
      linecollson.setFrameShadow(QtGui.QFrame.Sunken)

      lblsonar_alt              = QtGui.QLabel('Sonar Altimeter')
      lblsonaralt_port_pcas     = QtGui.QLabel('Port', self)

      linesonbut = QtGui.QFrame() # linea separatoria
      linesonbut.setFrameShape(QtGui.QFrame.HLine)
      linesonbut.setFrameShadow(QtGui.QFrame.Sunken)

      lblwinchdepth      = QtGui.QLabel('Winch Depth Info', self)
      lblwinchdepth_port = QtGui.QLabel('Port', self)

      linewinchdepth = QtGui.QFrame() # linea separatoria
      linewinchdepth.setFrameShape(QtGui.QFrame.HLine)
      linewinchdepth.setFrameShadow(QtGui.QFrame.Sunken)

      ## COMBO BOX ##


      # PCAS

      # select serial port

      # for profiler
      comboprofiler_port_pcas = QtGui.QComboBox(self)
      comboprofiler_port_pcas.setEditable(True)


      # connect to funcions
      comboprofiler_port_pcas.activated[str].connect(self.ComboProfiler_Port_Activated)


      # for camera
      combocollcamera_port_pcas = QtGui.QComboBox(self)
      combocollcamera_port_pcas.setEditable(True)

      # connect to funcions
      combocollcamera_port_pcas.activated[str].connect(self.ComboCamera_Port_Activated)
      # combocam_pow_channel.activated[str].connect(self.ComboCam_Pow_Activated)


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
          combowinchdepth.addItem("/dev/ttyUSB" + str(i))


      # connect to funcions
      combosonaralt_port_pcas.activated[str].connect(self.ComboAltimeter_Port_Activated)
      # comboalt_pow_channel.activated[str].connect(self.ComboAltimeter_Pow_Activated)
      combowinchdepth.activated[str].connect(self.ComboWinch_Port_Activated)

      ## CHECKBOX ##

      # PCAS

      # For power transmission selection

      self.cb_profiler_power = QtGui.QCheckBox('Power Up', self)
      self.cb_profiler_power.setChecked(True)
      self.cb_profiler_power.setToolTip('Power UP/DOWN Profiling Sonar Power (#0SON0/1)')

      self.cb_cam_power      = QtGui.QCheckBox('Power Up', self)
      self.cb_cam_power.setChecked(True)
      self.cb_cam_power.setToolTip('Power UP/DOWN PCAS Camera Power (#0CAM0/1)')

      self.cb_alt_power      = QtGui.QCheckBox('Power Up', self)
      self.cb_alt_power.setChecked(True)
      self.cb_alt_power.setToolTip('Power UP/DOWN Altimeter Power (#0ALT0/1)')

      # Callbacks

      self.cb_profiler_power.stateChanged.connect(self.ProfilerPowerCheckbox)
      self.cb_cam_power.stateChanged.connect(self.CameraPowerCheckbox)
      self.cb_alt_power.stateChanged.connect(self.AltimeterPowerCheckbox)

      # For data transmission selection

      self.cb_profiler_data = QtGui.QCheckBox('Data Transmission', self)
      self.cb_profiler_data.setToolTip('Switch Data transmission to Profiler (#0SSO0/1)')

      self.cb_cam_data      = QtGui.QCheckBox('Data Transmission', self)
      self.cb_cam_data.setToolTip('Switch between cameras to PCAS camera (#0VID0/1)')

      self.cb_alt_data      = QtGui.QCheckBox('Data Transmission', self)
      self.cb_alt_data.setToolTip('Switch Data transmission to Altimeter (#0SSO0/1)')

      # Callbacks

      self.cb_profiler_data.stateChanged.connect(self.ProfilerDataCheckbox)
      self.cb_cam_data.stateChanged.connect(self.CameraDataCheckbox)
      self.cb_alt_data.stateChanged.connect(self.AltimeterDataCheckbox)





      ## BUTTONS ##


      # For PCAS
      # Create button and set tooltip
      self.recordstr = 'RECORD'
      self.btnRECORD = QtGui.QPushButton(self.recordstr, self)
      self.btnRECORD.setToolTip('Button for START/STOP recording Profiling data')
      # Moving and resizing button. sizehint gives recommended size
      self.btnRECORD.resize(self.btnRECORD.sizeHint())
      #self.btnRECORD.setIcon(self.style().standardIcon(QStyle,'SP_DialogNoButton'))
      # Keep executing function while button pressed
      #self.btnRECORD.setAutoRepeat(True)

      self.btnRECORD.setStyleSheet('QPushButton {background-color: #E20202; color: white;}')
      self.btnRECORD.setFixedSize(110,50)
      self.btnRECORD.font().setBold(0)

      self.connect(self.btnRECORD, QtCore.SIGNAL("clicked()"),
                   self.RecordButtonActivated)




      ## PLAIN TEXT ##


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


      # For PCAS
      self.imgwinpcas = pg.GraphicsLayoutWidget()
      self.imgwinpcas.rotate(90.0)
      viewpcas = self.imgwinpcas.addViewBox()
      # Create image item
      self.collisioncameraimg = pg.ImageItem(border='w')
      viewpcas.addItem(self.collisioncameraimg)
      self.collision_ic = image_converter()

      # For sonar altimeter
      self.altimeter = altimeter_subscriber()


      ## POSITIONING ##


      layout2 = QtGui.QVBoxLayout()
      layout2.addWidget(lblobstacle)
      layout2.addWidget(self.StatusText2)
      layout2.addWidget(lbldepth)
      layout2.addWidget(self.DepthText)
      layout2.addWidget(linesonbut)
      layout2.addWidget(lblrecord)
      layout2.addWidget(self.btnRECORD)
      # Put it inside a QWidget so it can be added to QSplitter
      layoutw2 = QtGui.QWidget()
      layoutw2.setLayout(layout2)

      splitterpcas = QtGui.QSplitter(self)
      splitterpcas.addWidget(self.imgwinpcas)
      splitterpcas.addWidget(layoutw2)

      # Grouping sensor labels with their combo boxes

      layout1Hport_profiler = QtGui.QHBoxLayout()
      layout1Hport_profiler.addWidget(lblprofiler_port_pcas)
      layout1Hport_profiler.addWidget(comboprofiler_port_pcas)

      layout1Hport_collcamera = QtGui.QHBoxLayout()
      layout1Hport_collcamera.addWidget(lblcollcamera_port_pcas)
      layout1Hport_collcamera.addWidget(combocollcamera_port_pcas)

      layout1Hport_sonaralt = QtGui.QHBoxLayout()
      layout1Hport_sonaralt.addWidget(lblsonaralt_port_pcas)
      layout1Hport_sonaralt.addWidget(combosonaralt_port_pcas)

      layoutwinch_port = QtGui.QHBoxLayout()
      layoutwinch_port.addWidget(lblwinchdepth_port)
      layoutwinch_port.addWidget(combowinchdepth)


      #label vertical, with nav buttons and combo boxes
      layout1V2 = QtGui.QVBoxLayout()
      layout1V2.addWidget(linebr)
      layout1V2.addWidget(lblprofiling_sonar)
      layout1V2.addLayout(layout1Hport_profiler)
      # layout1V2.addLayout(layout1Hport_profiler2)
      # layout1V2.addLayout(layout1Hport_profiler3)
      layout1V2.addWidget(self.cb_profiler_power)
      layout1V2.addWidget(self.cb_profiler_data)
      layout1V2.addWidget(lineprofcoll)
      layout1V2.addWidget(lblcoll_camera)
      layout1V2.addLayout(layout1Hport_collcamera)
      # layout1V2.addLayout(layout1Hcamera)
      layout1V2.addWidget(self.cb_cam_power)
      layout1V2.addWidget(self.cb_cam_data)
      layout1V2.addWidget(linecollson)
      layout1V2.addWidget(lblsonar_alt)
      layout1V2.addLayout(layout1Hport_sonaralt)
      # layout1V2.addLayout(layout1Hport_sonaralt2)
      # layout1V2.addLayout(layout1Hport_sonaralt3)
      layout1V2.addWidget(self.cb_alt_power)
      layout1V2.addWidget(self.cb_alt_data)
      layout1V2.addWidget(linewinchdepth)
      layout1V2.addWidget(lblwinchdepth)
      layout1V2.addLayout(layoutwinch_port)

      # buttons + scatter widget
      layout1H2 = QtGui.QHBoxLayout()
      layout1H2.addWidget(splitterpcas)
      layout1H2.addLayout(layout1V2)

      self.setLayout(layout1H2)

      ## WINDOW DATA ##

      self.setGeometry(300, 300, 250, 150)
      self.setWindowTitle('Reference Points Installation Module')
      self.setWindowIcon(QtGui.QIcon('stams.png'))

      ## TOPIC SUBSCRIBERS ##

      # Subscribe to pose message that needs to be displayed
      self.altimeter.subscribe(self.altimeter.newsonarrange)

      self.collision_ic.subscribe(self.collision_ic.newcameraimage,"/v4l/bowtech_camera/image_raw")
      self.winchsubscriber = DepthSubscriber()
      self.winchsubscriber.start()

      ## SIGNALS ##

      # Create signal for the Plain text, connect to update_string function
      #self.ps.newstring.connect(self.update_string)
      # Signal for the new image received

      # for collision avoidance camera
      self.collision_ic.newcameraimage.connect(self.update_collisioncam_image)
      # For altimeter
      self.altimeter.newsonarrange.connect(self.update_string)
      # for depth board
      self.connect(self.winchsubscriber,self.winchsubscriber.signal,self.update_winch_depth)

      ## PARAMETER SERVER ##
      self.ProfilerParam = ParameterServer_Params()
      self.ProfilerParam.start()
      #self.profiler_client = dynamic_reconfigure.client.Client("/tritech_profiler")

      ## DISPLAY WIDGET ##
      # Display widget on screen
      self.show()

      self.showNormal()

    def update_winch_depth(self):
        """ Update the text which shows the information provided by the winch
        with the depth of the pcas
        """
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


    def Comboguimodeactivated(self, modetxt):
        if modetxt == 'Real ROV':
            self.ic.subscribe(self.ic.newcameraimage,"/BlueRov2/image")
        else:
            self.ic.subscribe(self.ic.newcameraimage,"/uwsim/camera1")

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

    def ComboCam_Pow_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.camera_pow_channel = text

    def ComboAltimeter_Pow_Activated(self,text):
        """This funcion is called when the Sonar altimeter combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Sonar altimeter is updated."""
        self.SDS_params.altimeter_pow_channel = text

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
        """
        Method that is executed everytime the state of the Checkbox for enabling/disabling Data transmission.
        When this checkbox is activated, the Altimeter checkbox is unselected.
        :param state: valiable that stores the state of the checkbox
        :return:

        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.profiler_data_enabled = 1
            self.cb_alt_data.setChecked(False)
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.profiler_channel, self.SDS_params.profiler_data_message)
            altimeter_params = {'altimeter_port_enabled': str(0)}
        else:
            self.SDS_params.profiler_data_enabled = 0

        profiler_params = {'port_enabled': str(self.SDS_params.profiler_data_enabled)}


        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update profiler params. Are you sure Profiler is connected?")

        try:
            config = self.ProfilerParam.valeport_altimeter_client.update_configuration(altimeter_params)
        except:
            rospy.logwarn("Could not update Altimeter params. Are you sure Altimeter is connected?")



    def AltimeterDataCheckbox(self,state):
        """
        Method that is executed everytime the state of the Altimeter Data Checkbox for enabling/disabling Data transmission changes.
        When this checkbox is activated, the Profiler checkbox is unselected.
        :param state: valiable that stores the state of the checkbox
        :return:

        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.altimeter_data_enabled = 1
            self.cb_profiler_data.setChecked(False)
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.altimeter_channel, self.SDS_params.altimeter_data_message)
            profiler_params = {'port_enabled': str(0)}
        else:
            self.SDS_params.altimeter_data_enabled = 0

        altimeter_params = {'altimeter_port_enabled': str(self.SDS_params.altimeter_data_enabled)}

        print altimeter_params

        try:
            config = self.ProfilerParam.valeport_altimeter_client.update_configuration(altimeter_params)
        except:
            rospy.logwarn("Could not update altimeter params. Are you sure Altimeter is connected?")

        try:
            config = self.ProfilerParam.profiler_client.update_configuration(profiler_params)
        except:
            rospy.logwarn("Could not update params. Are you sure Profiler is connected?")


    def CameraDataCheckbox(self,state):
        """
        Method that is executed everytime the state of the Camera Data Checkbox for enabling/disabling Data transmission changes.
        :param state: valiable that stores the state of the checkbox
        :return:

        """
        if state == QtCore.Qt.Checked:
            self.SDS_params.camera_data_enabled = 1
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.altimeter_channel, self.SDS_params.camera_data_message)
        else:
            self.SDS_params.camera_data_enabled = 0

    def RecordButtonActivated(self):
        """
        When activated, it starts recording a PointCloud, and when clicked again, it stops recording and saves the ply file.
        """
        if self.recordstr == 'RECORD':
            self.recordstr = 'STOP RECORD'
            self.btnRECORD.setText(self.recordstr)
            self.recordpoints = SavePointCloud()
            self.recordpoints.run()
        else:
            self.recordstr = 'RECORD'
            self.btnRECORD.setText(self.recordstr)
            self.recordpoints.subscriber.unregister()






def main():
    rospy.init_node('guilistener', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
