#!/usr/bin/python
import time
import math
import logging
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

usage = """Enter task number:
1 - Move forward
2 - Turn around
3 - Distance to object with color X
4 - Find object with color X"""


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
    relative_angle = angle*2*math.pi/360

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
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
        time.sleep(0.01) # todo: constants
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
 



def distance_to_color():
    pass


def find_object():
    pass


def main():
    rospy.init_node('cnc', anonymous=True)

    tasks_dict = {
        "1": move_forward,
        "2": turn_around,
        "3": distance_to_color,
        "4": find_object
    }

    while True:
        print(usage)
        txt = raw_input()
        if txt in tasks_dict:
            tasks_dict[txt]()
        else:
            print("Error: invalid task " + txt)


if __name__ == '__main__':
    main()
