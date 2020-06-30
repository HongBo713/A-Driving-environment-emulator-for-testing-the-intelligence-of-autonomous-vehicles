import os
import time
from UI_model import Ui_MainWindow
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
from PyQt5.QtCore import Qt
from pathlib import Path
from kalmanCTRA import downloadTurnLeft, downloadTurnRight
from main import getrealLocation


class GUiview(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(GUiview, self).__init__()
        self.setupUi(self)
        self.heading = 305
        self.pitch = 15
        self.index = 1
        self.pushButton.clicked.connect(self.start_on_click)  # add
        self.pushButton_2.clicked.connect(self.exit_on_click)  # add
        self.pushButton_3.clicked.connect(self.loading_on_clock)
        self.location = ''
        self.displayFlag = False


    def start_on_click(self):
        self.frame.setStyleSheet("#frame{ border-image: url('back2.jpg') 0 0 0 0 stretch stretch; "
                                 "background-size: ;}")
        self.label.deleteLater()
        self.pushButton.deleteLater()
        self.pushButton_2.deleteLater()
        self.textArea.setVisible(True)
        self.textArea.setGeometry(QtCore.QRect(380, 440, 101, 31))
        self.label2.setVisible(True)
        self.label2.setGeometry(QtCore.QRect(320, 440, 51, 31))
        self.label2.setText('<font color = "white">Location</font>')
        self.label3.setVisible(True)
        self.label3.setGeometry(QtCore.QRect(230, 30, 381, 281))
        font = QtGui.QFont()
        font.setPointSize(31)
        font.setBold(True)
        font.setWeight(75)
        self.label3.setText('<font color = "white">Initialization Page</font>')
        self.label3.setFont(font)
        self.pushButton_3.setVisible(True)
        self.pushButton_3.setGeometry(QtCore.QRect(350, 490, 81, 31))
        self.pushButton_3.setText("Loading")


    def loading_on_clock(self):
        self.progress.setVisible(True)
        self.progress.setGeometry(QtCore.QRect(320, 390, 201, 31))
        self.complete = 0
        while self.complete < 100:
            self.complete += 0.00001
            # self.complete += 0.000004
            self.progress.setValue(self.complete)
        self.label3.deleteLater()
        self.progress.deleteLater()
        self.textArea.deleteLater()
        self.pushButton_3.deleteLater()
        self.label2.deleteLater()
        self.location = self.textArea.text()  # get start position
        font2 = QtGui.QFont()
        font2.setPointSize(18)
        self.label4.setFont(font2)
        self.label4.setText(' Current speed: 50km/h')
        self.label4.setGeometry(QtCore.QRect(30, -100, 381, 281))
        self.label4.setVisible(True)
        # a,b = get_latAndLong(location)
        # # get_pic(locatioin, self.heading, self.pitch, 'download')
        # # download first 10 images, rename
        self.label4.setText(' Current speed: 0km/h')
        self.photo.setGeometry(QtCore.QRect(0, 0, 801, 581))
        self.photo.setPixmap(QtGui.QPixmap("downloads/1.jpg"))
        self.photo.setScaledContents(True)
        self.photo.setObjectName("forward")

    def getLocation(self):
        return self.location

    def walkFolder(self):
        return len(os.listdir("D:/360downloads/4th/downloads"))

    def updata_pic(self):
        print(' i am working on pic:' + str(self.index))
        # if self.walkFolder() > self.index+1:
        print('number of file: ' + str(self.walkFolder()))
        os.chdir("D:/360downloads/4th/downloads")
        if os.path.isfile(str(self.index) + ".jpg"):
            os.chdir('../')
            new_pic = QtGui.QPixmap("downloads/" + str(self.index) + ".jpg")
            self.photo.setPixmap(new_pic)
            self.photo.update()
            self.photo.setScaledContents(True)
            self.photo.setObjectName("forward")
        self.index += 1

        # else:
        #     time.sleep(1)


    def exit_on_click(self):
        sys.exit()

    def keyPressEvent(self, event):
        # if only up, play image
        # if left or right, clear buffer, download image, rename, play image
        key = event.key()
        if key == Qt.Key_Up:
            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self.updata_pic)
            self.timer.start(600)
            self.label4.setText(' Current speed: 50km/h')
        elif key == Qt.Key_Left:
            coordinate = getrealLocation()

            downloadTurnLeft(coordinate[0],coordinate[1],self.heading)
            # self.v.set_signal('left')
            # self.v.setSignalIndex(self.index)
        elif key == Qt.Key_Right:
            coordinate = getrealLocation()
            downloadTurnLeft(coordinate[0],coordinate[1],self.heading)
            downloadTurnRight()
            # self.v.set_signal('right')
            # self.v.setSignalIndex(self.index)
        elif key == Qt.Key_B:
            self.timer.stop()
            self.timer.timeout.connect(self.updata_pic)
            self.timer.start(800)
            self.label4.setText(' Current speed: 35km/h')
        elif key == Qt.Key_S:
            self.timer.stop()
            self.timer.timeout.connect(self.updata_pic)
            self.timer.start(400)
            self.label4.setText(' Current speed: 75km/h')
