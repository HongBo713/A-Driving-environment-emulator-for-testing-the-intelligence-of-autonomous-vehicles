import time
from UI_control import GUiview
from main import downloadpredict
from PyQt5 import QtWidgets
import sys
import threading
import os


path = "downloads/"
n = 0
startLocation= ''
def fatherThread():
    a = 1

def display():
    print("start working: " + str(threading.current_thread()))
    app = QtWidgets.QApplication(sys.argv)
    window = GUiview()
    startLocation = window.getLocation()
    window.show()
    app.exec_()

def downloadpath(n):

    if startLocation == '':
        time.sleep(1)
    else:
        downloadpredict(startLocation,GUiview.heading)


threads = []
t1 = threading.Thread(target=fatherThread, args=())
threads.append(t1)
t2 = threading.Thread(target=display,args=())
threads.append(t2)
t3 = threading.Thread(target=downloadpath, args=(n,))
threads.append(t3)
for t in threads:
    t.start()