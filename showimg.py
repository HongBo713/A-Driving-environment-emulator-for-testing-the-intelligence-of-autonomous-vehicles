import cv2
import glob
import numpy as np


def show(count):
    # img_data = []
    # files = glob.glob("C:/Users/baoti/4th/downloads/*.jpg")
    #
    # for myFile in files:
    #     print(myFile)
    #     img = cv2.imread(myFile)
    #     img_data.append(img)
    img = cv2.imread("C:/Users/baoti/4th/downloads/" + str(count) +".jpg")
    cv2.imshow("imgs",img)
    cv2.waitKey(0)

#img = cv2.imread("1.jpg",0)
#resized_imge = img[200:700, 500:1100]
#img2 = cv2.imread("2.jpg",0)
#resized_imge2 = img[200:700, 500:1100]
    # init = 1
    # i = 1

    # while(1):
    #
    #     k = cv2.waitKey(33)
    #     if init == 1:
    #         cv2.imshow("imgs",img_data[0])
    #         cv2.moveWindow("imgs", 20,20);
    #         init = 0
    #     if k == 32:
    #         cv2.destroyWindow('imgs')
    #         cv2.imshow("imgs2",img_data[i])
    #         cv2.moveWindow("imgs2", 20,20)
    #         i = i + 1;
    #     if k== 27:    # Esc key to stop
    #         break

        #cv2.destroyAllWindows()
