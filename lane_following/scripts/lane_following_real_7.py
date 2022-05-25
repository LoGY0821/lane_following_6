#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image', Image,
                                          self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.twist = Twist()
        self.stopped = 0
        self.left = 0
        self.right = 0
        self.goin = 0
        self.go = 0

    def move(self, err, mode):
        if mode == 0:
            # full speed forward
            self.twist.linear.x = 0.1
            self.twist.angular.z = err * 2
        elif mode == 1:
            # turn left
            self.twist.linear.x = 0.1 * (1 - err)
            self.twist.angular.z = err * 2
        elif mode == 2:
            # turn right
            self.twist.linear.x = 0.1 * (1 - err)
            self.twist.angular.z = err * 2
        self.cmd_vel_pub.publish(self.twist)

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def aruco_call(self, image):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        param = aruco.DetectorParameters_create()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, markerID, rejected = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          parameters=param)
        matrix = numpy.array([[322.0704122808738, 0., 199.2680620421962],
                              [0., 320.8673986158544, 155.2533082600705],
                              [0., 0., 1.]])
        dist = numpy.array([[
            0.1639958233797625, -0.271840030972792, 0.001055841660100477,
            -0.00166555973740089, 0.
        ]])

        if len(corners) > 0 and self.stopped != 1:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.0125, matrix, dist)
            (rvec - tvec).any()
            for i in range(rvec.shape[0]):
                aruco.drawDetectedMarkers(image, corners, markerID)
                #corners=markerCorner.reshape((4,2))
                #(topLeft,topRight,bottomRight,bottomleft)=corners
                #topRight = (int(topRight[0]), int(topRight[1]))
                #bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                #bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                #topLeft = (int(topLeft[0]), int(topLeft[1]))
                #cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                #cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                #cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                #cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            distance = int(tvec[0][0][2] * 1000)
            print("[INFO] ArUco marker ID: {}".format(markerID))
            print("distance: ", distance, "mm")
            if distance <= 100:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                self.stopped = 1
                time.sleep(10)

    def image_callback(self, msg):
        # HSV black range
        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([180, 255, 46])
        # Projection array
        ori = numpy.array([[8, 233], [312, 233], [78, 180], [245, 180]])
        dst = numpy.array([[80, 240], [220, 240], [90, 80], [220, 80]])

        # image reading
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape

        # hsv masking
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # make projection
        hinfo, status = cv2.findHomography(ori, dst)
        mask1 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask2 = cv2.warpPerspective(mask, hinfo, (w, h))
        mask = cv2.warpPerspective(mask, hinfo, (w, h))

        #make area mask
        mask1[0:h, 0:w / 2] = 0
        mask2[0:h, w / 2:w] = 0

        # pixel calculation
        M1 = cv2.moments(mask)
        M2 = cv2.moments(mask1)
        M3 = cv2.moments(mask2)

        # process
        if M1['m00'] > 0:
            # have the image, calculate the moment
            cx1 = int(M1['m10'] / M1['m00'])
            cy1 = int(M1['m01'] / M1['m00'])
            cv2.circle(mask, (cx1, cy1), 10, (255, 255, 255), -1)
            err = (w / 2 - cx1) * 2 / w
            if M3['m00'] > 0:
                cx3 = int(M3['m10'] / M3['m00'])
                cy3 = int(M3['m01'] / M3['m00'])
                cv2.circle(mask, (cx3, cy3), 5, (255, 255, 255), -1)
                if M2['m00'] > 0:
                    cx2 = int(M2['m10'] / M2['m00'])
                    cy2 = int(M2['m01'] / M2['m00'])
                    cv2.circle(mask, (cx2, cy2), 5, (255, 255, 255), -1)
                    if self.right == 1:
                        if self.go == 2:
                            print("turn right")
                            self.move(err, 2)
                            time.sleep(0.5)
                        else:
                            print("has right but forward")
                            self.go = self.go + 1
                            self.move(err, 0)
                    elif cy3 > 200:
                        if self.goin == 0:
                            if cy2>180:
                                self.move(err,0)
                            else:
                                print("start detect left")
                                self.left = 1
                        else:
                            print("start running")
                            self.move(err, 2)
                            time.sleep(0.5)
                            self.goin = 1
                    else:
                        self.move(err, 0)
                elif self.left == 1:
                    print("turn left")
                    self.move(err, 1)
                    time.sleep(0.5)
                    self.left = 0
                elif self.right == 0:
                    print("start detect right")
                    self.right = 1
                else:
                    print("empty case1. ")
                    self.stop()
            elif M2['m00'] <= 0:
                print("try to start")
                self.goin = 1
            else:
                print("empty case2. ")

        # if M3['m00'] > 0 and M2['m00'] <= 0:
        #     self.twist.linear.x = 0.17
        #     self.twist.angular.z = 0
        #     print("No right side. ")
        #     self.cmd_vel_pub.publish(self.twist)
        # else:
        #     cx1 = int(M1['m10'] / M1['m00'])
        #     cy1 = int(M1['m01'] / M1['m00'])

        #     cv2.circle(image, (cx1, cy1), 10, (0, 0, 255), -1)

        #     err = w / 2 - cx1
        #     print("Forward")
        #     self.twist.linear.x = 0.17
        #     self.twist.angular.z = (err * 90.0 / 160) / 15
        #     self.cmd_vel_pub.publish(self.twist)
        else:
            print("no image, stop")
            self.stop()
        # self.aruco_call(self, image)
        cv2.imshow("window", image)
        cv2.imshow("window2", mask)
        cv2.imshow("window3", mask1)
        cv2.imshow("window4", mask2)
        cv2.waitKey(1)


rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
