#!/usr/bin/env python
#coding:utf-8
import rospy
import sys
import tty
import termios
from geometry_msgs.msg import PoseStamped

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

    rospy.init_node('state_machine', anonymous=True)

    goal_pub = rospy.Publisher('/goal_zph_ugv1', PoseStamped, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    goal = PoseStamped()

    goal.header.stamp = rospy.Time.now()

    goal.header.frame_id = "map"

    
    key=readkey()
    if key=='y':
        goal.pose.position.x = 7
        goal.pose.position.y =24
    if key=='u':
        goal.pose.position.x =11
        goal.pose.position.y = 24
    # if key=='p':
    #     break
    #     #go_left()
    # if key=='d':
    #     #go_right()
    # if key=='q':
           

        

    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0

    goal.pose.orientation.y = 0.0

    goal.pose.orientation.z = 0.0

    goal.pose.orientation.w = 1.0


    goal_pub.publish(goal)

    rate.sleep()


if __name__ == '__main__':

    try:

        while not rospy.is_shutdown():

            print("Enter 'y' for goal 1, 'u' for goal 2 ")


            send_goal()


    except rospy.ROSInterruptException:

        pass