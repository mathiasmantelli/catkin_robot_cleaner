#!/usr/bin/env python

import rospy
import roslib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


def my_video_publisher():
    video_capture = cv2.VideoCapture('../video/tennis-ball-video.mp4')
    bridge = CvBridge() #an object that makes the transformation between opencv and ros
    rospy.init_node('tennis_ball_publisher', anonymous=True)
    video_publisher = rospy.Publisher("tennis_ball_image", Image, queue_size=10)
    loop_rate = rospy.Rate(25) # we publish the velocity at 10 Hz (10 times a second)    

    while(True):
        ret, frame = video_capture.read()
        try:
            video_publisher.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)        
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #frame = cv2.resize(frame, (0,0), fx=0.5,fy=0.5)
        #cv2.line(frame,(0,0),(511,511),(255,0,0),5)
        #cv2.imshow("Frame",frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break
        loop_rate.sleep()

    video_capture.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    my_video_publisher()