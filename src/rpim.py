#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys
sys.path.append("/home/olaya/V4LOG_to_point_cloud")
from v4log_to_scatter import V4LOG_to_py

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import numpy as np
import scipy.ndimage as ndi
# For subscribers
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
# For the thrusters publisher
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud

import pcl

"""
.. codeauthor:: Olaya Alvarez Tunon
: file rov_ui.py
"""

class MyGLView(gl.GLViewWidget):
    """ *MyGLView* class inherits from GLViewWidget class, and overwrites the *paintGL* and *mousePresEvent* methods
    to provide personalised capabilities.
    This class provides the same capabilities than GLViewWidget, but additionally it allows to add text in the 3D
    plot and calls a callback fuction every time the mouse clicks on the widget.
    """
    def paintGL(self, *args, **kwds):
        """
        *paintGL* method draws over the widget every time it is refreshed. Here the original class is overwriten in order
        to render text over it
        :param args: receives a GLViewWidget object
        :param kwds: receives a GLViewWidget object called "region", it is the area we are writting on.
        :return:
        """
        gl.GLViewWidget.paintGL(self, *args, **kwds)
        self.qglColor(QtCore.Qt.white)
        self.renderText(0, 0, 0, 'origin')


    def mousePressEvent(self,ev):
        """
        *mousePressEvent* method overwrites the function from the parent class. This method is called every time
        the mouse is clicked on the associated widget.
        :param ev: the event that executes this class.
        :return:
        """
        self.mousePos = ev.pos()
        print "pressed button HERE",ev.button(),"at", ev.pos()



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
      self.mutex = QtCore.QMutex(mode = QMutex.Recursive)
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

      ## TOOLTIPS ##

      # Font used to render tooltips, and text displayed
      QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
      self.setToolTip('This is a <b>QWidget</b> widget')

      ## LABELS ##

      # For ROV

      lblgui = QtGui.QLabel('Scene selection', self)
      lblnav = QtGui.QLabel('Navigation Mode', self)

      # For PCAS
      lblnavpcas = QtGui.QLabel('Speed & Direction control', self)

      ## COMBO BOX ##
      guimode = QtGui.QComboBox(self)
      guimode.addItem("Simulator")
      guimode.addItem("Real ROV")

      navmode = QtGui.QComboBox(self)
      navmode.addItem("Teleoperated")
      navmode.addItem("Autonomous")


      ## JOYSTICK ##
      self.jb = pg.JoystickButton()

      self.jb.setFixedWidth(120)
      self.jb.setFixedHeight(120)
      region = QtGui.QRegion(QtCore.QRect(self.jb.x()+5,
                                      self.jb.y()+5
                                       ,110
                                      ,110)
                                      ,QtGui.QRegion.Ellipse)
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

      # Create plain text
      self.StatusText = QtGui.QPlainTextEdit("Waiting for first message")
      # With this, the user wont be able to edit plain text
      self.StatusText.setReadOnly(1)
      # Scroll automatically to the bottom of the text
      self.StatusText.centerCursor()

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
      self.pcas = MyGLView()
      self.pcas.addItem(g)
      pointplot = gl.GLScatterPlotItem(pos=real_pose, color=(1, 1, 1, 1), size=0.1)
      self.pcas.addItem(pointplot)

      ## TABS ##
      tab_widget = QtGui.QTabWidget()
      tab1 = QtGui.QWidget()
      tab2 = QtGui.QWidget()

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

      #label vertical
      layout1V2 = QtGui.QVBoxLayout()
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
      self.ps.newstring.connect(self.update_string)
      self.ps.newscatter.connect(self.update_pose_plot)
      # Signal for the new image received
      # for ROV
      self.ic.newcameraimage.connect(self.update_ROV_image)
      # for collision avoidance camera
      self.collision_ic.newcameraimage.connect(self.update_collisioncam_image)

      # Profiling Sonar PointCloud
      self.PS_PC = ProfilingSonar_PC()
      self.connect(self.PS_PC, self.PS_PC.newpcsignal, self.update_PS_PointCloud)
      self.PS_PC.start()

      ## TOPIC SUBSCRIBERS ##

      # Subscribe to pose message that needs to be displayed
      self.ps.subscribe(self.ps.newstring,self.ps.newscatter)
      self.ic.subscribe(self.ic.newcameraimage,"/uwsim/camera1")
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

        verts = np.array([[0,0,0]])
        for point in self.PS_PC.profiler_PC.points:
            verts = np.vstack((verts, [point.x,point.y,point.z]))

        color = np.ones((len(verts), 4))

        self.scatt_prof = gl.GLScatterPlotItem(pos=verts,color=color,size=5)
        self.scatt_prof.setGLOptions('translucent')
        self.pcas.addItem(self.scatt_prof)

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
        self.StatusText.appendPlainText('z =' + self.ps.datastring)

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

def main():
    rospy.init_node('guilistener', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
