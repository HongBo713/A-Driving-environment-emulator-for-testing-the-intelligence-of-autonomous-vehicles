from diffimg import diff
from os import path
import os

def checkDiff(img1,img2):

    os.chdir('C:/Users/baoti/4th/downloads')
    differnce = diff(img1,img2,delete_diff_file=False, diff_img_file="DIFF_IMG_FILE.jpg")
    os.chdir('../')
    return differnce



def delete(img_name):
    os.chdir('C:/Users/baoti/4th/downloads')
    if path.exists(img_name):
        os.remove(img_name)
    os.chdir('../')



print(checkDiff("0.jpg","1.jpg"))
print(checkDiff("0.jpg","2.jpg"))
# print (diff('44.jpg','45.jpg'))
