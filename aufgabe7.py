import cv2

import roslib

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import math

import sys

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import rospy

import time

from std_msgs.msg import UInt8

from std_msgs.msg import Int16

from std_msgs.msg import Float32

from nav_msgs.msg import Odometry

from std_msgs.msg import String



class Line_Detection:

    def __init__(self):
        self.startTime = rospy.get_time()
        self.angleArray = []
        self.image_pub = rospy.Publisher("/line_detection/bin_img", Image, queue_size=1)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)

        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)

        self.pub_stop_start = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100)

        #self.pub_stop_start.publish(0)

        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size = 100)

        #rospy.sleep(1)

        #self.speed_pub.publish(120)



       # self.speed_sub = rospy.Subscriber("/manual_control/speed", Int16, self.callback_driving, queue_size=1)



    def callback(self, data):

        try:

            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

            bi_gray_max = 255

            bi_gray_min = 200

            ret,self.cv_image=cv2.threshold(self.cv_image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
            self.cv_image = self.cv_image[100:]
            self.ransac(self.cv_image)

            #print(points)

        except CvBridgeError as e:

            print(e)



        try:

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "mono8"))

        except CvBridgeError as e:

            print(e)



    def callback_driving(self, data):

        print("drive")

        drive(data)



    def ransac(self, image):

        b1 = 0

        s1 = 0

        rounds = 0

        best_model = None

        # Ransac parameters

        ransac_ratio = 0.3  # ratio of inliers required to assert

        ransac_bestRatio = 0.0

        while best_model == None and rounds < 10:

           # print(rounds)

            rounds += 1

            points = []

            t = 0

           # while True:

            y = np.random.randint(image.shape[0])

            x = np.random.randint(image.shape[1])

               # if (image[y, x] == 255):

            points.append([y, x])

            y1 = np.random.randint(image.shape[0])

            x1 = np.random.randint(image.shape[1])

            points.append([y1, x1])





            #        if (len(points) == 2):

             #           break

            # 0) Punkte in normales Koordinatensystem umrechnen

            # pointsNormal = [[image.shape[0] - points[0][0], points[0][1]],

            #                [image.shape[0] - points[1][0], points[1][1]]]

            # 1) Steigung berechnen

            if ((points[0][1] - points[1][1]) == 0):

                m = 9999

            else:

                m = (points[0][0] - points[1][0]) / float(points[0][1] - points[1][1])

            # 2) y-Achsenabschnitt berechnen

            t = points[1][0] - (m * points[1][1])

            # 3) Gerade berechnen

            # print(image.sh0[1])

            whiteCounter = 0

            blackCounter = 0

            pixelCounter = 0

            for i in range(0, image.shape[1]):

                y1 = int(m * i + t)

               # y1 = image.shape[0] - y1

                if (y1 > image.shape[0] - 1 or y1 < 0):

                    continue

                else:

                    pixelCounter += 1

                    if (image[y1, i] == 255):

                        whiteCounter += 1

                    else:

                        blackCounter += 1



            if (whiteCounter > 0):

                ratio = whiteCounter/float(pixelCounter)

                if (ratio >= ransac_ratio and ratio > ransac_bestRatio):

                    ransac_bestRatio = ratio

                    best_model = points

                    b1 = t

                    s1 = m

        # return best_model_negative, best_model_positive, int(b1), int(s1), int(b2), int(s2)

        if(best_model != None):

            self.drive(b1, s1, best_model, image)

            return best_model, int(b1), int(s1)

        return 0, 0, 0



    def drive(self, b, s, best_model, image):

        ys = image.shape[0] -  150

        #print(image.shape[1])

        self.pub_stop_start.publish(0)
        self.speed_pub.publish(0)
        #self.speed_pub.publish(100)
        self.pub_stop_start.publish(1)

        xs = (ys - b) / float(s)
        xcar = float(image.shape[1]) / 2
        ycar = image.shape[0]
        angle = np.arctan((xcar-xs)/-(ycar-ys))
        #to deg
        angle = np.clip(90 + int(np.rad2deg(angle)), 30, 150)
        self.angleArray.append([angle,int(rospy.get_time()- self.startTime)])
        self.pub_steering.publish(angle)





def main(args):

    rospy.init_node('line_Detection', anonymous=True)

    line_Detection = Line_Detection()

    try:

      rospy.spin()

    except KeyboardInterrupt:

      print("Shutting down")
    angleArray = np.array(line_Detection.angleArray)
    f = plt.figure()
    plt.plot(angleArray[:,1],angleArray[:,0])
    plt.show()
    f.savefig("f.pdf", bbox_inches = "tight")
    print(angleArray)
    cv2.destroyAllWindows()





if __name__ == '__main__':

     main(sys.argv)
