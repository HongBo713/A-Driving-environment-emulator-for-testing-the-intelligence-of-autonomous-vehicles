from array import array

from kalman import *
from showimg import *
from plot import *
from imgDiff import *
from pathlib import Path
from central import *

import time
import cv2
import os
import json
diffx = 0
diffy = 0
count = 0
k = 0
turn = 1
location = ""
realLocation = ""
file = Path('C:/Users/baoti/4th/downloads')


def download(coord, heading):
    global count
    lat = str(coord[0])
    long = str(coord[1])
    location = lat + "," + long

    get_pic(location, heading, '0', str(count))
    count += 1
    print(count)

    if count > 1:
        img1 = str(count-1) + ".jpg"
        img2 = str(count - 2) + ".jpg"
        print(img1)
        print(img2)
        os.chdir('C:/Users/baoti/4th/downloads')
        if path.exists(img1) and path.exists(img2):

            print("the difference between " + img1 + " and " + img2 + " is " +
                  str(checkDiff(img1, img2)))
            if (checkDiff(img1, img2) > 0.15) or (checkDiff(img1, img2) == 0):
                delete(img1)
                count -= 1

        os.chdir('../')



def getrealLocation():
    os.chdir('C:/Users/baoti/4th/downloads')
    with open('metadata.json', 'r') as myfile:
        data = myfile.read()
    dataphrase = json.loads(data)
    os.chdir('../')
    if str(dataphrase[0]['status']) == "ZERO_RESULTS":
        return ("no result")
    else:
        realLat = dataphrase[0]["location"]['lat']
        realLong = dataphrase[0]["location"]['lng']
        return (realLat, realLong)


def downloadpredict(output, heading):
    global diffx, diffy
    for i in range(len(output)):
        print("------------------------------------------")
        download(output[i], heading)
        print("Prediction : " + str(output[i]))
        print("Real :       " + str(getrealLocation()))
        print("------------------------------------------")
        diffx  += (getrealLocation()[0] - output[i][0])
        diffy  += (getrealLocation()[1] - output[i][1])

def downloadimg(lat, long, heading):
    global k
    global  trun
    output = KalmanCalculation(0, 0, lat, long, heading)
    #downloadpredict(output, heading)
    print (output)
    plot(output)

    while(1):
        newlat = output[-1][0]
        newlong =  output[-1][1]
        refresh()
        output = KalmanCalculation(0, 0, newlat, newlong, heading)
        print("------------------------this is " + str(k) + " set-----------------------")
        downloadpredict(output,heading)
        #break

    refresh()


def keePdownloadimg(lat, long, heading):
    global k
    global  trun
    output = KalmanCalculation(0, 0, lat, long, heading)
    downloadpredict(output, heading)

    for i in range(1):
        newlat = output[-1][0]
        newlong =  output[-1][1]
        refresh()
        output = KalmanCalculation(0, 0, newlat, newlong, heading)
        print("------------------------this is " + str(k) + " set-----------------------")
        downloadpredict(output,heading)
        #break

    refresh()

# downloadimg(45.3747403, -75.7092024,240.5)
# avX = diffx/10
# avy = diffy/10
# print(avX,avy)

#get_pic("e", 96.19, '0', "0")
#get_pic("45.375664, -75.699303", 96.19, '0', "1")
#get_pic("45.3757613,-75.6991076", 96.19, '0', "2")