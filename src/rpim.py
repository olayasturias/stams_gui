#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar

import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np


import pyqtgraph as pg
import pyqtgraph.opengl as gl

import scipy.ndimage as ndi
# For subscribers
#import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
# For the thrusters publisher
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Range

import serial

"""
.. codeauthor:: Olaya Alvarez Tunon
: file rpim.py
"""

class SDS_Params():
    def __init__(self):

        self.baudrate = 57600
        self.previous_baudrate = 57600

        self.profiler_channel  = '/dev/ttyUSB0'
        self.camera_channel    = '/dev/ttyUSB0'
        self.altimeter_channel = '/dev/ttyUSB0'

        self.profiler_pow_channel  = 'A'
        self.camera_pow_channel    = 'A'
        self.altimeter_pow_channel = 'A'

        self.profiler_pow_enabled   = 0
        self.camera_pow_enabled     = 0
        self.altimeter_pow_enabled  = 0

        self.profiler_data_channel  = '1'
        self.camera_data_channel    = '1'
        self.altimeter_data_channel = '1'

    def parse_params(self):
        self.profiler_pow_message  = '+++' + str(self.profiler_pow_channel) + str(self.profiler_pow_enabled) +' ;'
        self.camera_pow_message    = '+++' + str(self.camera_pow_channel)   + str(self.camera_pow_enabled)   + ';'
        self.altimeter_pow_message = '+++' + str(self.altimeter_pow_channel)+ str(self.altimeter_pow_enabled)+ ';'

        self.profiler_data_message  = '+++' + str(self.profiler_data_channel) + ';'
        self.camera_data_message    = '+++' + str(self.camera_data_channel)   + ';'
        self.altimeter_data_message = '+++' + str(self.altimeter_data_channel)+ ';'

        self.baudrate_message       = '+++' + 'M' + str('C' if (self.baudrate == 115200) else self.baudrate/9600)+';'

    def send(self, port, message):
        ser = serial.Serial(port,self.baudrate)
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
        # Emit the signal associated
        signal.emit()
      except CvBridgeError as e:
        print(e)
      #cv2.imshow("Image window", self.cv_image)
      #cv2.waitKey(3)

class pose_subscriber(QObject):
    """ The *pose_subscriber* class contains methods to subscribe to the Pose
    topic , read the messages and save them in a float variable and in a
    string variable.

    **Attributes**:

        .. data:: newstring

            Signal used for drawing a new string read from topic in a PlainText

        .. data:: newscatter

            Signal used for drawing a new pose point in a 3D scatter plot

        .. data:: pose_world_frame

            float variable which stores the x,y,z coordinates of the robot pose

        .. data:: datastring

            string variable which stores the x coordinate of the robot pose

        .. data:: datainput

        this variables is used for writing or drawing a new pose message only
        when it differs from the previous one

        """

    newstring = pyqtSignal()
    newscatter = pyqtSignal()

    def subscribe(self,signal1,signal2):
        """ Subscribes to a pose topic
        """
        self.datainput = [0,0]
        rospy.Subscriber("/g500/pose", Pose, self.pose_callback, (signal1,signal2))

    def pose_callback(self,data,(signal1,signal2)):
        """ Reads data from pose topic and emits the signals related with this message
        """
        # Read data from topic
        self.pose_world_frame = [data.position.x, data.position.y, data.position.z]
        self.datainput[0] = round(self.pose_world_frame[2],2)

        # Emit signal to update string when new data is received from the topic
        # and only if the value has changed
        if (self.datainput[0] != self.datainput[1]):
          self.datainput[1] = self.datainput[0]
          # format data to be displayed to string
          self.datastring = "{:.2f}".format(self.pose_world_frame[2])
          # Emit signal
          signal1.emit()
          signal2.emit()

class altimeter_subscriber(QObject):

    newsonarrange = pyqtSignal()

    def subscribe(self, signal1):
        " Subscribes to range topic "
        self.range = 0
        self.datastring = ''
        rospy.Subscriber("/range", Range, self.altimeter_callback, (signal1))

    def altimeter_callback(self,data,signal1):
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

class ProfilingSonar_PC(QThread):
    """ *ProfilingSonar_PC* executes a thread where the point cloud
    from the Profiling Sonar sensor is read, and the signal is emitted to
    refresh the GUI.
    """
    def __init__(self):
        ''' Initialize the Point Cloud drawing thread.'''
        QtCore.QThread.__init__(self)
        self.newpcsignal = QtCore.SIGNAL("PSsignal")

        rospy.Subscriber("/tritech_profiler/scan",PointCloud,self.PS_callback)
    #
    # def run(self):
    #     ''' This function is executed every ___ seconds, and the pointcloud plot signal
    #     is emitted'''
    #     while 1:
    #         time.sleep(1)
    #
    #         self.PSCloud = V4LOG_to_py('PS.CSV')
    #         self.PSCloud.generate_useful_data_header()
    #         self.PSCloud.read_useful_csv('PS.CSV')
    #         self.PSCloud.convert_cylindrical_cartesian()
    #
    #         self.emit(self.newpcsignal,"profilingsonar PC thread")
    def PS_callback(self,data):
        self.profiler_PC = data
        self.emit(self.newpcsignal,"profilingsonar PC thread")


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

      lblgui = QtGui.QLabel('Scene selection', self)
      lblnav = QtGui.QLabel('Navigation Mode', self)

      # For PCAS
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

      ## COMBO BOX ##

      # ROV

      guimode = QtGui.QComboBox(self)
      guimode.addItem("Simulator")
      guimode.addItem("Real ROV")

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
      comboprofiler_data_channel.addItem("1")
      comboprofiler_data_channel.addItem("2")

      # connect to funcions
      comboprofiler_data_channel.activated[str].connect(self.ComboProfiler_Data_Activated)

      # for camera
      combocollcamera_port_pcas = QtGui.QComboBox(self)
      combocollcamera_port_pcas.setEditable(True)

      # connect to funcions
      combocollcamera_port_pcas.activated[str].connect(self.ComboCamera_Port_Activated)


      # for altimeter
      combosonaralt_port_pcas = QtGui.QComboBox(self)
      combosonaralt_port_pcas.setEditable(True)

      # Add labels to combo box options
      for i in range (0, 4):
          comboprofiler_port_pcas.addItem(  "/dev/ttyUSB" + str(i))
          combocollcamera_port_pcas.addItem("/dev/ttyUSB" + str(i))
          combosonaralt_port_pcas.addItem(  "/dev/ttyUSB" + str(i))


      comboalt_pow_channel = QtGui.QComboBox(self)
      comboalt_pow_channel.addItem("A")
      comboalt_pow_channel.addItem("B")
      comboalt_pow_channel.addItem("C")

      # connect to funcions
      combosonaralt_port_pcas.activated[str].connect(self.ComboAltimeter_Port_Activated)
      comboalt_pow_channel.activated[str].connect(self.ComboAltimeter_Pow_Activated)

      comboalt_data_channel = QtGui.QComboBox(self)
      comboalt_data_channel.addItem("1")
      comboalt_data_channel.addItem("2")

      # connect to funcions
      comboalt_data_channel.activated[str].connect(self.ComboAltimeter_Data_Activated)

      ## CHECKBOX ##

      # PCAS

      # For power transmission selection

      self.cb_profiler_power = QtGui.QCheckBox('Power Up', self)
      #cb_cam_power      = QtGui.QCheckBox('Power Up', self)
      self.cb_alt_power      = QtGui.QCheckBox('Power Up', self)

      # Callbacks

      self.cb_profiler_power.stateChanged.connect(self.ProfilerPowerCheckbox)
      #cb_cam_power.stateChanged.connect(self.CameraPowerCheckbox)
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
      btnSTOPpcas.setFixedSize(220,110)
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

      self.StatusText2 = QtGui.QPlainTextEdit("Waiting for first message")
      # With this, the user wont be able to edit plain text
      self.StatusText2.setReadOnly(1)
      # Scroll automatically to the bottom of the text
      self.StatusText2.centerCursor()

      ## IMAGE VIEW ##

      # Create window with GraphicsView widget
      self.imgwin = pg.GraphicsLayoutWidget()
      view = self.imgwin.addViewBox()
      # Create image item
      self.img = pg.ImageItem(border='w')
      view.addItem(self.img)
      self.ic = image_converter()

      # For PCAS
      self.imgwinpcas = pg.GraphicsLayoutWidget()
      self.imgwinpcas.rotate(90.0)
      viewpcas = self.imgwinpcas.addViewBox()
      # Create image item
      self.collisioncameraimg = pg.ImageItem(border='w')
      viewpcas.addItem(self.collisioncameraimg)
      self.collision_ic = image_converter()

      ## 3D GRAPICS ##

      # For pose displaying

      self.ps = pose_subscriber()
      self.win = gl.GLViewWidget()
      # Add a grid to the view
      g = gl.GLGridItem()
      g.setDepthValue(10)  # draw grid after surfaces since they may be translucent
      self.win.addItem(g)
      # Add Scatter Plot to widget
      real_pose = np.zeros((1,1,3))
      poseplot = gl.GLScatterPlotItem(pos=real_pose,color=(1,1,1,1),size=0.1)
      self.win.addItem(poseplot)

      # For PointCloud from PCAS displaying

      # Here I use my class because I wanted to use the mouseclick callback in this widget
      #self.pcas = MyGLView()
      #self.pcas.addItem(g)
      self.pcasfigure = plt.figure()
      self.pcas = FigureCanvas(self.pcasfigure)

      self.toolbar = NavigationToolbar(self.pcas, self)
      self.toolbar.hide()
      self.toolbar.zoom()

      # For sonar altimeter
      self.altimeter = altimeter_subscriber()


      ## TABS ##
      tab_widget = QtGui.QTabWidget()
      tab1       = QtGui.QWidget()
      tab2       = QtGui.QWidget()

      ## POSITIONING ##

      splitter = QtGui.QSplitter(self)
      splitter.addWidget(self.imgwin)
      splitter.addWidget(self.win)
      splitter.addWidget(self.StatusText)

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

      # Add to the buttons label "navigation" vertically
      layoutV3 = QtGui.QVBoxLayout()
      layoutV3.addWidget(lblgui)
      layoutV3.addWidget(guimode)
      layoutV3.addWidget(lblnav)
      layoutV3.addWidget(navmode)

      # Mode selection with navigation controllers
      layoutV4 = QtGui.QVBoxLayout()
      layoutV4.addLayout(layoutV3)
      layoutV4.addLayout(layoutH1)

      # LayoutV1 and LayoutV2 horizontal
      layoutH2 = QtGui.QHBoxLayout(tab1)
      layoutH2.addLayout(layoutV1)
      layoutH2.addLayout(layoutV4)

      # Layout of PCAS tab

      splitterpcas = QtGui.QSplitter(self)
      splitterpcas.addWidget(self.pcas)
      splitterpcas.addWidget(self.imgwinpcas)
      splitterpcas.addWidget(self.StatusText2)

      # BtnUP and btnDOWN vertical between them (with joystick), form layoutV2
      layout1V1 = QtGui.QVBoxLayout()
      layout1V1.addWidget(btnUPpcas)
      layout1V1.addWidget(btnSTOPpcas)
      layout1V1.addWidget(btnDOWNpcas)

      #BtnRIGHT, layoutV2 and btnLEFT placed horizontally, form LayoutH1
      layout1H1 = QtGui.QHBoxLayout()
      layout1H1.addWidget(btnLEFTpcas)
      layout1H1.addLayout(layout1V1)
      layout1H1.addWidget(btnRGTpcas)

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

      layout1Hport_sonaralt = QtGui.QHBoxLayout()
      layout1Hport_sonaralt.addWidget(lblsonaralt_port_pcas)
      layout1Hport_sonaralt.addWidget(combosonaralt_port_pcas)

      layout1Hport_sonaralt2 = QtGui.QHBoxLayout()
      layout1Hport_sonaralt2.addWidget(lblalt_pow_channel)
      layout1Hport_sonaralt2.addWidget(comboalt_pow_channel)

      layout1Hport_sonaralt3 = QtGui.QHBoxLayout()
      layout1Hport_sonaralt3.addWidget(lblalt_data_channel)
      layout1Hport_sonaralt3.addWidget(comboalt_data_channel)


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
      layout1V2.addWidget(linecollson)
      layout1V2.addWidget(lblsonar_alt)
      layout1V2.addLayout(layout1Hport_sonaralt)
      layout1V2.addLayout(layout1Hport_sonaralt2)
      layout1V2.addLayout(layout1Hport_sonaralt3)
      layout1V2.addWidget(self.cb_alt_power)
      layout1V2.addWidget(self.cb_alt_data)
      layout1V2.addWidget(linesonbut)
      layout1V2.addWidget(lblnavpcas)
      layout1V2.addLayout(layout1H1)

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

      ## SIGNALS ##

      # Create signal for the Plain text, connect to update_string function
      #self.ps.newstring.connect(self.update_string)
      self.ps.newscatter.connect(self.update_pose_plot)
      # Signal for the new image received
      # for ROV
      self.ic.newcameraimage.connect(self.update_ROV_image)
      # for collision avoidance camera
      self.collision_ic.newcameraimage.connect(self.update_collisioncam_image)
      # For altimeter
      self.altimeter.newsonarrange.connect(self.update_string)

      # Profiling Sonar PointCloud
      self.PS_PC = ProfilingSonar_PC()
      self.connect(self.PS_PC, self.PS_PC.newpcsignal, self.update_PS_PointCloud)
      self.PS_PC.start()

      ## TOPIC SUBSCRIBERS ##

      # Subscribe to pose message that needs to be displayed
      self.ps.subscribe(self.ps.newstring,self.ps.newscatter)
      self.altimeter.subscribe(self.altimeter.newsonarrange)
      self.collision_ic.subscribe(self.collision_ic.newcameraimage,"/v4l/bowtech_camera/image_raw")

      ## DISPLAY WIDGET ##
      # Display widget on screen
      self.show()

    def update_PS_PointCloud(self):
        """ This function updates the point cloud every time the signal is emitted from the
        *PointCloud_PC* thread. The point cloud showed is the result from the Profiling Sonar Scanning.
        **Attributes**:

        .. data: verts
        The points read from the script that converts the data from the sensor to a scatter to be read from this GUI

        .. data: color
        The color associated to each point from the scatter. The color varies with the Z coordinate

        .. data: color_normed
        The normalised matrix for the color of each point: the values must be between 0 and 1

        .. data: scatt_prof
        the GLScatterPlotItem which contains the point cloud with the new mesaruements read

        .. data: scatt_plot_label
        A GLScatterPLotItem which is used to show text in the widget.
        """

        # z = self.PS_PC.PSCloud.cylindric_cartesian['Z']
        # x = self.PS_PC.PSCloud.cylindric_cartesian['X']
        # y = self.PS_PC.PSCloud.cylindric_cartesian['Y']
        #
        #
        # # Normalize values (to create the color palette)
        # z_norm = z/z.max()
        # # Extract the sensor values to be drawn
        # verts = np.empty((len(x), 3), dtype=np.float64)
        # verts[:,0] = x
        # verts[:,1] = y
        # verts[:,2] = z*20
        # # Create the color palette
        # color = np.ones((len(x),4))
        # color[:,0] = 6.25*z_norm*z_norm*z_norm-9.375*z_norm*z_norm+2.125*z_norm+1
        # color[:,1] = 4.17*z_norm*z_norm*z_norm-6.25*z_norm*z_norm+3.1*z_norm
        # color[:,2] = 4.17*z_norm*z_norm*z_norm-6.25*z_norm*z_norm+3.1*z_norm
        # #color[:,3] = 4.17*z_norm*z_norm*z_norm-6.25*z_norm*z_norm+3.1*z_norm
        #
        # color_normed = color/color.max(axis = 0)
        #
        # # Draw the scatter plot
        # self.scatt_prof = gl.GLScatterPlotItem(pos=verts,color=color_normed,size=5)
        # self.scatt_prof.setGLOptions('translucent')
        # self.pcas.addItem(self.scatt_prof)
        #
        # # Add label with mouse cursor values
        # self.scatt_plot_label = MyGLView(self.pcas)
        # self.scatt_plot_label.paintGL(region=self.pcas.getViewport())

        x = np.array(0)
        y = np.array(0)
        z = np.array(0)
        for point in self.PS_PC.profiler_PC.points:
            x = np.append(x,point.x)
            y = np.append(y,point.y)
            z = np.append(z,point.z)

        #color = np.ones((len(verts), 4))
        tri = mtri.Triangulation(x, y)

        ax = self.pcasfigure.add_subplot(111,projection='3d')
        self.pcasfigure.tight_layout()
        ax.set_axis_bgcolor('black')
        black = (0,0,0,0)
        ax.w_xaxis.set_pane_color(black)
        ax.w_yaxis.set_pane_color(black)
        ax.w_zaxis.set_pane_color(black)
        ax.xaxis.label.set_color('lightgray')
        ax.yaxis.label.set_color('lightgray')
        ax.zaxis.label.set_color('lightgray')
        ax.tick_params(axis='x', colors='white')
        ax.tick_params(axis='y', colors='white')
        ax.tick_params(axis='z', colors='white')
        ax.hold(False)
        ax.plot_trisurf(x,y,z,triangles=tri.triangles, linewidth=0.2, antialiased=True)
        ax.auto_scale_xyz([-500,500],[-500,500],[10,-500])
        self.pcas.draw()

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
        #pub = rospy.Publisher("/g500/thrusters_input", Float64MultiArray, queue_size=10)
        #msg = Float64MultiArray()
        thrusters[0]=thrusters[1]=dy
        thrusters[4]=-dx
        #msg.data = thrusters
        #pub.publish(msg)


    def update_string(self):
        ''' This function updates the text value fron the PlainText widget every time the
        value changes
        '''
        self.StatusText.appendPlainText('obstacle =' + self.altimeter.datastring)
        self.StatusText2.appendPlainText('obstacle =' + self.altimeter.datastring)

    def update_pose_plot(self):
        ''' This function updates the pose of the ROV and adds it to its trajectory in the
        3D scatter plot of the GUI
        '''
        pos3 = np.zeros((1,1,3))
        pos3[0,0,:] = self.ps.pose_world_frame
        poseplot = gl.GLScatterPlotItem(pos=pos3,color=(1,1,1,1))
        self.win.addItem(poseplot)

    def update_ROV_image(self):
        '''Every time a new image topic arrives, it is updated in the gui widget with this function
        '''
        self.img.setImage(self.ic.cv_image)


    def update_collisioncam_image(self):
        '''Every time a new image topic arrives, it is updated in the gui widget with this function
        '''
        self.collisioncameraimg.setImage(self.collision_ic.cv_image)


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

    def ComboBaudActivated(self, baudtxt):

        self.SDS_params.previous_baudrate = self.SDS_params.baudrate
        self.SDS_params.baudrate = int(baudtxt)

        self.SDS_params.parse_params()

        self.SDS_params.change_baudrate()


    def ComboProfiler_Port_Activated(self,text):
        self.SDS_params.profiler_channel = text

    def ComboCamera_Port_Activated(self,text):
        self.SDS_params.camera_channel = text

    def ComboAltimeter_Port_Activated(self,text):
        self.SDS_params.altimeter_channel = text

    def ComboProfiler_Pow_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.profiler_pow_channel = text

    def ComboProfiler_Data_Activated(self,text):
        """This funcion is called when the Profiling Sonar combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Profiling sonar is updated."""
        self.SDS_params.profiler_data_channel = text

    def ComboAltimeter_Pow_Activated(self,text):
        """This funcion is called when the Sonar altimeter combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Sonar altimeter is updated."""
        self.SDS_params.altimeter_pow_channel = text

    def ComboAltimeter_Data_Activated(self,text):
        """This funcion is called when the Sonar altimeter combo box for selection of Power Supply Channel is activated.
        Here the value saved in the SDS_Params class for the Power supply channel of the Sonar altimeter is updated."""
        self.SDS_params.altimeter_data_channel = text

    def ProfilerPowerCheckbox(self,state):
        if state == QtCore.Qt.Checked:
            self.SDS_params.profiler_pow_enabled = 1
        else:
            self.SDS_params.profiler_pow_enabled = 0

        self.SDS_params.parse_params()
        self.SDS_params.send(self.SDS_params.profiler_channel, self.SDS_params.profiler_pow_message)


    def AltimeterPowerCheckbox(self,state):
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
        else:
            self.SDS_params.profiler_data_enabled = 0


    def AltimeterDataCheckbox(self,state):
        if state == QtCore.Qt.Checked:
            self.SDS_params.altimeter_data_enabled = 1
            self.cb_profiler_data.setChecked(False)
            self.SDS_params.parse_params()
            self.SDS_params.send(self.SDS_params.altimeter_channel, self.SDS_params.altimeter_data_message)
        else:
            self.SDS_params.altimeter_data_enabled = 0





def main():
    rospy.init_node('guilistener', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
