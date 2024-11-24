#!/usr/bin/env python
#coding:utf-8
import rospy
import sys
import tty
import termios
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray,Float32


def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)

def send_goal():
    
    pub1_pose = rospy.Publisher('/blue/ugv1/state_topic', Float32, queue_size=10)
    pub2_pose = rospy.Publisher('/blue/ugv2/state_topic', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    key=readkey()
    if key=='q':#hold
        pub1_pose.publish(11)
    
    if key=='a':#hold
        pub2_pose.publish(11)
    if key=='w':#hold
        pub1_pose.publish(12)
    
    if key=='s':#hold
        pub2_pose.publish(12)

    if key=='e':#hold
        pub1_pose.publish(13)
    
    if key=='d':#hold
        pub2_pose.publish(13)

    rate.sleep()


if __name__ == '__main__':

    try:
        rospy.init_node('state_machine_sim')
        while not rospy.is_shutdown():

            print("Enter 'qwe' for car 1, 'asd' for car 2 ")
            send_goal()
        # rospy.spin()
            


    except rospy.ROSInterruptException:

        pass
