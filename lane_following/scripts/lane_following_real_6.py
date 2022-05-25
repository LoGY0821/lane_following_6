#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
                self.stopped=False

        def image_callback(self, msg):
                lower_black=numpy.array([0,0,0])
                upper_black=numpy.array([180,255,30])

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                
                h, w, d = image.shape
                
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower_black, upper_black)
                
                ori = numpy.array([[8, 233], [312,233], [78, 180],[245, 180]])
                dst = numpy.array([[80, 240], [220,240], [90, 80],[220, 80]])
                # ori = numpy.array([[8, 233], [312,233], [70, 190],[250, 190]])
                # dst = numpy.array([[80, 240], [220,240], [90, 80],[220, 80]])
                
                hinfo, status = cv2.findHomography(ori, dst)
                mask1 = cv2.warpPerspective(mask, hinfo, (w,h))
                mask2 = cv2.warpPerspective(mask, hinfo, (w,h))
                mask3 = cv2.warpPerspective(mask, hinfo, (w,h))
                mask4 = cv2.warpPerspective(mask, hinfo, (w,h))
                mask5 = cv2.warpPerspective(mask, hinfo, (w,h))
                mask = cv2.warpPerspective(mask, hinfo, (w,h))

                mask1[0:h, w/2:w] = 0 #left

                mask4[0:2*h/3, 0:w] = 0 #right down
                mask4[0:h, 0:w/2] = 0 #

                mask3[0:h/3, 0:w] = 0 #right middle
                mask3[2*h/3:h, 0:w] = 0 #
                mask3[0:h, 0:w/2] = 0

                mask2[0:h, 0:w/2] = 0 #right up
                mask2[h/3:h, 0:w] = 0

                mask5[0:h, w/5:w] = 0 #left_left
                

                M  = cv2.moments(mask)
                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)
                M3 = cv2.moments(mask3)
                M4 = cv2.moments(mask4)
                M5 = cv2.moments(mask5)

                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        if M1['m00']>0 :
                                cx1 = int(M1['m10']/M1['m00'])
                                cy1 = int(M1['m01']/M1['m00'])
                                cv2.circle(mask1, (cx1, cy1), 10, (255,255,255), -1)
                                if cy1>100 and cy1<140 :
                                        if M5['m00']>0 and (M2['m00']<=0  or M3['m00']<=0 or M4['m00']<=0)  :#
                                                self.twist.linear.x = 0.17
                                                self.twist.angular.z = (err*90.0/160)/80
                                                print("No right side. Go Foward")
                                                # self.cmd_vel_pub.publish(self.twist)
                                        else:
                                                cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
                                                err = w/2 - cx
                                                print("Normal")
                                                self.twist.linear.x = 0.17
                                                self.twist.angular.z = (err*90.0/160)/15
                                                # self.cmd_vel_pub.publish(self.twist)
                                else:
                                        cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
                                        err = w/2 - cx
                                        print("Normal")
                                        self.twist.linear.x = 0.17
                                        self.twist.angular.z = (err*90.0/160)/15
                                        # self.cmd_vel_pub.publish(self.twist)

                # if M['m00'] > 0:
                #         if M1['m00']>0 and M2['m00']<=0:
                #                 self.twist.linear.x = 0.17
                #                 self.twist.angular.z = 0
                #                 print("No right side. ")
                #                 self.cmd_vel_pub.publish(self.twist)
                #         else:
                #                 cx1 = int(M['m10']/M['m00'])
                #                 cy1 = int(M['m01']/M['m00'])

                #                 cv2.circle(image, (cx1, cy1), 10, (0,0,255), -1)

                #                 err = w/2 - cx1
                #                 print("Forward")
                #                 self.twist.linear.x = 0.17
                #                 self.twist.angular.z = (err*90.0/160)/15
                #                 self.cmd_vel_pub.publish(self.twist)

                        
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                param = aruco.DetectorParameters_create()
                
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                corners, markerID, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=param)
                matrix = numpy.array([[322.0704122808738, 0., 199.2680620421962],
                                      [0., 320.8673986158544, 155.2533082600705],
                                      [0., 0., 1.]])
                dist = numpy.array([[0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0.]]) 
                
                if len(corners)>0 and not self.stopped: 
                        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.0125, matrix, dist)
                        (rvec-tvec).any()
                        for i in range(rvec.shape[0]):
                                aruco.drawDetectedMarkers(image, corners,markerID)
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
                        distance = int(tvec[0][0][2]*1000)
                        print("[INFO] ArUco marker ID: {}".format(markerID))
                        print("distance: ", distance, "mm")
                        if distance<=100:
                                self.twist.linear.x=0
                                self.twist.angular.z=0
                                self.cmd_vel_pub.publish(self.twist)
                                self.stopped=True
                                time.sleep(10)

                cv2.imshow("window", image)
                cv2.imshow("window2",mask)
                cv2.imshow("window3",mask1)
                cv2.imshow("window4",mask2)
                cv2.imshow("window5",mask3)
                cv2.imshow("window6",mask4)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
