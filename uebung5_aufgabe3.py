import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
from PIL import Image


image = cv2.imread("./bild.jpg")	#Lädt Bild 
#print(pixels.shape)
image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
pixels = np.array(image) 
#plt.imshow(image, "gray")
#plt.show()

def ransac():
    b1 = 0
    s1 = 0
    b2 = 0
    s2 = 0
    rounds = 0    
    best_model_negative = None
    best_model_positive = None
    # Ransac parameters
    ransac_ratio = 0.5                           # ratio of inliers required to assert
    ransac_bestRatio_p = 0                       
    ransac_bestRatio_n = 0 
    while best_model_negative == None or best_model_positive == None or rounds < 500:
        rounds += 1
        points = []
        t = 0
        while True:
            y = np.random.randint(image.shape[0])
            x = np.random.randint(image.shape[1])
            if(image[y,x] == 255):
                points.append([y,x])
                if(len(points) == 2):
                    break
        # 0) Punkte in normales Koordinatensystem umrechnen
        pointsNormal = [[image.shape[0]-points[0][0], points[0][1]],[image.shape[0]-points[1][0], points[1][1]]]
        # 1) Steigung berechnen
        if((pointsNormal[0][1]-pointsNormal[1][1]) == 0):
            m = 1
        else:
            m = (pointsNormal[0][0]-pointsNormal[1][0]) / (pointsNormal[0][1]-pointsNormal[1][1])
        # 2) y-Achsenabschnitt berechnen
        t = pointsNormal[1][0] - (m * pointsNormal[1][1] + t)
        # 3) Gerade berechnen
        #print(image.shape[1])
        whiteCounter = 0
        blackCounter = 0
        pixelCounter = 0
        for i in range (0, image.shape[1]-1):
            y1 = int(m*i +t)
            y1 = image.shape[0]-y1
            if(y1 > image.shape[0]-1 or y1 < 0):
                continue
            else:
                pixelCounter += 1
                if(image[y1,i] == 255):
                    whiteCounter += 1
                else:
                    blackCounter += 1
        if(m < 0):
            print("n")
            if(whiteCounter > 0):
                if(whiteCounter/pixelCounter >= ransac_ratio and whiteCounter/pixelCounter > ransac_bestRatio_n):
                    ransac_bestRatio = (pixelCounter*whiteCounter)/100
                    best_model_negative = points
                    b1 = t
                    s1 = m
        else:
            print("p")
            if(whiteCounter > 0):
                if(whiteCounter/pixelCounter >= ransac_ratio and whiteCounter/pixelCounter > ransac_bestRatio_p):
                    ransac_bestRatio = (pixelCounter*whiteCounter)/100
                    best_model_positive = points
                    b2 = t
                    s2 = m
    return best_model_negative, best_model_positive, int(b1), int(s1), int(b2), int(s2)

bm = ransac()
print(bm)
image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
cv2.line(image, (bm[1][0][1],bm[1][0][0]) , (bm[1][1][1],bm[1][1][0]), (0,0,255),3)
cv2.line(image, (bm[0][0][1],bm[0][0][0]) , (bm[0][1][1],bm[0][1][0]), (255,0,0),3)
cv2.imshow("image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
print("Steigung und Y-Achsenabschnitt von rot: ", bm[4], bm[3])
print("Steigung und Y-Achsenabschnitt von blau: ", bm[6], bm[5])
print(image.shape)