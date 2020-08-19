import cv2 
import numpy as np   
import imutils 
# This drives the program into an infinite loop. 
def blue(img): 
    lower_blue = np.array([160,0,0]) 
    upper_blue = np.array([255,200,100]) 
    img = cv2.resize(img,(400,300)) 
    mask = cv2.inRange(img, lower_blue, upper_blue) 
    res = cv2.bitwise_and(img,img, mask= mask) 
    #edges = cv2.Canny(res,100,200) 
    gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0,255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    areas = [cv2.contourArea(c) for c in cnts]
    max_index = np.argmax(areas)
    c=cnts[max_index]
    M = cv2.moments(c)
    cv2.imshow('color',thresh) 
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX-200
    except:
        return 0       

img = cv2.imread('img.png')
px_dist = blue(img)
print(px_dist)
cv2.waitKey(0) & 0xFF 
