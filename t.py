



#!/usr/bin/python
import time
import math
import logging
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys 
from geometry_msgs.msg import Vector3

#hsv values
#red 0 - 15
#green 60- 90
#blue 90-120


bridge = CvBridge()

colordict = {

	"red": [ [0, 50, 50], [15, 255, 255]],
	"green": [ [60, 50, 50], [90, 255, 255]],
	"blue": [ [90, 50, 50], [120, 255, 255]]
}
def distance_to_color():
	color_input = raw_input("Enter color: ")
	print(color_input)
    
	image_sub = rospy.Subscriber("/camera/image_raw", Image, callback3, (color_input))
        rospy.spin()


def callback3( image, color ):

	print (color)

	color_range = 0

	#get color hsv range
	if color in colordict:
		color_range = colordict[color]
	else:
		print("Error: invalid color " + color)
		return null

	
	try:
        	cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
    	except CvBridgeError as e:
    		print(e)


	#print(color_range)

	c_min = np.array( color_range[0] )
	c_max = np.array( color_range[1] )

	hsv_img = cv2.cvtColor( cv_image, cv2.COLOR_BGR2HSV )


	frame_threshed = cv2.inRange( hsv_img, c_min, c_max )
	#cv2.imwrite( 'output2.jpg', frame_threshed )

	M = cv2.moments(frame_threshed)
	 
	if M["m00"]:
		# calculate x,y coordinate of center
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	else:
		cX = null
		cy = null

	print( cX, cY )
	return ( cX, cY )

def main():
    rospy.init_node('cnc', anonymous=True)
    while True:
        distance_to_color()

if __name__ == '__main__':
    main()
