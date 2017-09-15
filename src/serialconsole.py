#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys

from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QObject, pyqtSignal, QRunnable, QThread,QThreadPool
from PyQt4.Qt import QObject, QMutex, QApplication, QThread, QMutexLocker, QEvent,QMouseEvent

import serial

class SerialSubscribe(QThread):
    def __init__(self, channel):
        QtCore.QThread.__init__(self)
        self.channel = channel
        self.signal = QtCore.SIGNAL("signal")

    def run(self):
        while 1:
            time.sleep(0.01)
            try:
                self.line = self.channel.readline()
                self.emit(self.signal, "line read")
            except:
                self.line = 'could not read stream. Try restarting port'
                pass


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

        ## LABELS ##
        lblsent = QtGui.QLabel('Last command sent:')
        lblrec = QtGui.QLabel('Received:')
        lblport = QtGui.QLabel('Port:', self)
        lblbr = QtGui.QLabel('BaudRate:', self)
        lbltimeout = QtGui.QLabel('Timeout:', self)
        lblstatus = QtGui.QLabel('Status:')
        self.lblst =QtGui.QLabel('OK')

        self.OpenClosePort()

        try:
            self.serconn = serial.Serial(self.port, self.baudrate)
        except:
            self.lblst.setText('could not open port. Device connected?')

        ## BTN for dialog ##
        self.btndialog = QtGui.QPushButton('Send serial command', self)
        self.btndialog.resize(self.btndialog.sizeHint())
        self.btndialog.clicked.connect(self.showDialog)

        ## btn for open and closing port
        self.btnport = QtGui.QPushButton('Open/Close')
        self.btnport.setStyleSheet('background-color: red')
        self.connect(self.btnport, QtCore.SIGNAL("clicked()"),
                     self.OpenClosePort)

        # Sent messages text

        self.le = QtGui.QLineEdit(self)
        self.le.resize(self.le.sizeHint())

        # received messages text

        self.re = QtGui.QPlainTextEdit(self)
        self.re.resize(self.le.sizeHint())
        # With this, the user wont be able to edit plain text
        self.re.setReadOnly(1)
        # Scroll automatically to the bottom of the text
        self.re.centerCursor()


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


        combotimeout = QtGui.QSpinBox(self)
        combotimeout.setMinimum(-1)
        combotimeout.setMaximum(1000000)
        combotimeout.setSingleStep(2)
        combotimeout.setValue(2)
        combotimeout.setSuffix('s')

        combotimeout.valueChanged.connect(self.comboTimeout)


        layoutV1 = QtGui.QVBoxLayout()
        layoutV1.addWidget(self.btndialog)
        layoutV1.addWidget(lblsent)
        layoutV1.addWidget(self.le)
        layoutV1.addWidget(lblrec)
        layoutV1.addWidget(self.re)

        layoutV2 = QtGui.QVBoxLayout()
        layoutV2.addWidget(lblport)
        layoutV2.addWidget(comboport)
        layoutV2.addWidget(lblbr)
        layoutV2.addWidget(combolbr)
        layoutV2.addWidget(lbltimeout)
        layoutV2.addWidget(combotimeout)


        layoutH1 = QtGui.QHBoxLayout()
        layoutH1.addLayout(layoutV1)
        layoutH1.addLayout(layoutV2)

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
        self.ssub.port = str(text)
        rospy.set_param('sds_port', self.port)
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def comboBaudActivated(self, baudtxt):
        self.baudrate = int(baudtxt)
        self.serconn.baudrate = self.baudrate
        self.ssub.baudrate = self.baudrate
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def OpenClosePort(self):

        try:
            openport = self.serconn.is_open
        except:
            try:
                self.serconn = serial.Serial(self.port,self.baudrate)
            except:
                self.lblst.setText('could not open port. Device connected?')
            openport = self.serconn.is_open

        if openport:
            self.serconn.close()

        else:
            self.serconn.open()


        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
            self.ssub = SerialSubscribe(self.serconn)
            self.ssub.start()
            self.connect(self.ssub, self.ssub.signal, self.update_text)
        else:
            self.lblst.setText('Serial port is closed')

    def comboTimeout(self,text):
        self.serconn.timeout = int(text)
        self.ssub.timeout = int(text)
        if self.serconn.is_open:
            self.lblst.setText('Serial port is open')
        else:
            self.lblst.setText('Serial port is closed')

    def update_text(self):
        if self.ssub.line != '':
            self.re.appendPlainText(self.ssub.line)





def main():
    rospy.init_node('serialport', anonymous=True)
    # Create application object
    app = QtGui.QApplication(sys.argv)
    ex = Window()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
