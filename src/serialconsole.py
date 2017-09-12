#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent

import serial

class Window(QtGui.QWidget):
    def __init__(self):
        # The super method returns the parent object of the Windw class, and we call its constructor
        super(Window, self).__init__()
        # Creation of the gui
        self.initUI()


    def initUI(self):

        ## Variables for serial communication ##
        self.port = '/dev/ttyUSB0'
        self.baudrate = 57600
        self.timeout = 1000
        self.serconn = serial.Serial(self.port, self.baudrate)

        ## LABELS ##
        lblsent = QtGui.QLabel('Last command sent:')
        lblrec = QtGui.QLabel('Received:')
        lblport = QtGui.QLabel('Port:', self)
        lblbr = QtGui.QLabel('BaudRate:', self)
        lbltimeout = QtGui.QLabel('Timeout (ms):', self)
        lblstatus = QtGui.QLabel('Status:')
        self.lblst =QtGui.QLabel('okey makey')

        ## BTN for dialog ##
        self.btndialog = QtGui.QPushButton('Send serial command', self)
        self.btndialog.resize(self.btndialog.sizeHint())
        self.btndialog.clicked.connect(self.showDialog)

        ## btn for open and closing port
        self.btnport = QtGui.QPushButton('Open/Close')
        self.btnport.setStyleSheet('background-color: red')
        self.connect(self.btnport, QtCore.SIGNAL("clicked()"),
                     self.OpenClosePort)

        self.le = QtGui.QLineEdit(self)
        self.le.resize(self.le.sizeHint())

        self.re = QtGui.QLineEdit(self)
        self.re.resize(self.le.sizeHint())

        ##  Combo Box ##

        comboport = QtGui.QComboBox(self)
        comboport.addItem("/dev/ttyUSB0")
        comboport.addItem("/dev/ttyUSB1")
        comboport.addItem("/dev/ttyUSB2")
        comboport.addItem("/dev/ttyUSB3")
        comboport.addItem("/dev/ttyUSB4")

        comboport.activated[str].connect(self.comboPortActivated)

        combolbr = QtGui.QComboBox(self)
        combolbr.addItem("115200")
        combolbr.addItem("57600")
        combolbr.addItem("38400")
        combolbr.addItem("28800")
        combolbr.addItem("19200")
        combolbr.addItem("14400")
        combolbr.addItem("9600")

        combolbr.activated[str].connect(self.comboBaudActivated)

        combotimeout = QtGui.QComboBox(self)
        combotimeout.setEditable(True)

        combotimeout.activated[str].connect(self.comboTimeout)



        layoutV1 = QtGui.QVBoxLayout()
        layoutV1.addWidget(self.btndialog)
        layoutV1.addWidget(lblsent)
        layoutV1.addWidget(self.le)
        layoutV1.addWidget(lblrec)
        layoutV1.addWidget(self.re)

        layoutV2 = QtGui.QVBoxLayout()
        layoutV2.addWidget(lblport)
        layoutV2.addWidget(lblbr)
        layoutV2.addWidget(lbltimeout)

        layoutV3 = QtGui.QVBoxLayout()
        layoutV3.addWidget(comboport)
        layoutV3.addWidget(combolbr)
        layoutV3.addWidget(combotimeout)


        layoutH1 = QtGui.QHBoxLayout()
        layoutH1.addLayout(layoutV1)
        layoutH1.addLayout(layoutV2)
        layoutH1.addLayout(layoutV3)

        layoutH2 = QtGui.QHBoxLayout()
        layoutH2.addWidget(lblstatus)
        layoutH2.addWidget(self.lblst)
        layoutH2.addWidget(self.btnport)

        main_layout = QtGui.QVBoxLayout()
        main_layout.addLayout(layoutH1)
        main_layout.addLayout(layoutH2)


        self.setLayout(main_layout)

        ## WINDOW DATA ##

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Serial Communication Console')

        ## DISPLAY WIDGET ##
        # Display widget on screen
        self.show()

    def showDialog(self):
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                                  'Enter serial command:')

        if ok:
            self.serconn.write(str(text))
            self.le.setText(str(text))

    def comboPortActivated(self,text):
        self.port = str(text)
        self.serconn.port = str(text)
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def comboBaudActivated(self, baudtxt):
        self.baudrate = int(baudtxt)
        self.serconn.baudrate = self.baudrate
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def OpenClosePort(self):
        if self.serconn.is_open:
            self.serconn.close()
        else:
            self.serconn.open()

        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def comboTimeout(self,text):
        self.serconn.timeout = int(text)
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')









def main():
    rospy.init_node('serialport', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()