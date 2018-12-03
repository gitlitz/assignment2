#!/usr/bin/python
import time
import math
import logging
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

USAGE = """Enter task number:
1 - Move forward
2 - Turn around
3 - Distance to object with color X
4 - Find object with color X"""

# TODO CAMERA_FOV = 1.3962634 * 180 / math.pi  # based on turtlebot3_burger_cam.gazebo.xacro
CAMERA_FOV = 1.3962634 # based on turtlebot3_burger_cam.gazebo.xacro

COLOR_THRESHOLDS = {
    "red": [np.array([0, 50, 50]), np.array([15, 255, 255])],
    "green": [np.array([60, 50, 50]), np.array([90, 255, 255])],
    "blue": [np.array([90, 50, 50]), np.array([120, 255, 255])]
}


def distance_to_front():
    data = rospy.wait_for_message("/scan", LaserScan)
    center = data.ranges[0]
    print("distance to center: " + str(center))
    return center


def move_forward():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while distance_to_front() > 0.5:
        msg = Twist()
        msg.linear.x = 0.4
        pub.publish(msg)

    msg = Twist()
    msg.linear.x = 0.0
    pub.publish(msg)


def turn_around():
    raw = raw_input("Enter how many degrees you want to turn: ")
    try:
        angle = float(raw)
    except ValueError:
        print("Error: {} is not a valid number: ".format(raw))
        return

    angular_speed = 2.5
    relative_angle = angle * 2 * math.pi / 360

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    vel_msg.angular.z = angular_speed
    if angle < 0:
        vel_msg.angular.z = -angular_speed
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while abs(current_angle) < abs(relative_angle):
        velocity_publisher.publish(vel_msg)
        time.sleep(0.01)  # todo: constants
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def distance_to_color():
    pass


def angle_to_color(color):
    """
    :return: the angle to the center of an object in the given color. None if no object is available for the camera
    """
    image = rospy.wait_for_message("/camera/image_raw", Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")

    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    if color not in COLOR_THRESHOLDS:
        return None

    frame_threshed = cv2.inRange(hsv_img, COLOR_THRESHOLDS[color][0], COLOR_THRESHOLDS[color][1])

    moment = cv2.moments(frame_threshed)

    if not moment["m00"]:
        return None  # object in the given color was not found

    x = int(moment["m10"] / moment["m00"])

    # find the angle
    image_width = cv_image.shape[1]
    l = image_width / 2 / math.tan(CAMERA_FOV / 2)
    angle = math.atan((x - image_width / 2) / l)

    # convert to degrees
    return angle * 180 / math.pi

def find_object():
    print(angle_to_color("green"))


def main():
    rospy.init_node('cnc', anonymous=True)

    tasks_dict = {
        "1": move_forward,
        "2": turn_around,
        "3": distance_to_color,
        "4": find_object
    }

    while True:
        print(USAGE)
        txt = raw_input()
        if txt in tasks_dict:
            tasks_dict[txt]()
        else:
            print("Error: invalid task " + txt)


if __name__ == '__main__':
    main()
