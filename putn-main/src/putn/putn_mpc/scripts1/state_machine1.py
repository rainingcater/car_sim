#!/usr/bin/env python3
#coding:utf-8
import rospy
import sys
import tty
import termios
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray,Float32
from  local_planner.msg import BlueModeDef
from  local_planner.msg import BlueMode
from  local_planner.msg import RedModeDef
from  local_planner.msg import RedMode
from sensor_msgs.msg import Temperature
model_index=13
uav1_health_value = 100
uav2_health_value = 100

def health_callback_uav1(data):
    global uav1_health_value
    uav1_health_value = data.temperature  
    # rospy.loginfo(f"UAV1 Health Value: {uav1_health_value}")

def health_callback_uav2(data):
    global uav2_health_value
    uav2_health_value = data.temperature  
    # rospy.loginfo(f"UAV2 Health Value: {uav2_health_value}")

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


def ugv0_state_callback(data):
    # rospy.loginfo("Received Pose: %s", data)
    return data


def send_goal():

    rospy.init_node('state_machine', anonymous=True)

    goal_pub = rospy.Publisher('/goal_zph_ugv1', PoseStamped, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    goal = PoseStamped()

    goal.header.stamp = rospy.Time.now()

    goal.header.frame_id = "map"

    global model_index
    print('zph')
    print(model_index)
    if model_index==11:#hold
        if uav2_health_value!=0 and uav1_health_value!=0:
            print('HOLD1')
            goal.pose.position.x =5
            goal.pose.position.y =15
        else:
            print('HOLD2')
            goal.pose.position.x =4
            goal.pose.position.y =15 
    if model_index==12:#BLUE_PURSUIT
        print('BGLUPERSULT')
        goal.pose.position.x =blue_uav1_pose. pose.position.x
        goal.pose.position.y =blue_uav1_pose.pose.position.y
        print(goal.pose.position.x)
        print(goal.pose.position.y)
    if model_index==13:# LURE_DEEP
        if uav2_health_value!=0 and uav1_health_value!=0:
            print('luredeep1')
            goal.pose.position.x =4
            goal.pose.position.y = 13
        else:
            print('luredeep2')
            goal.pose.position.x =3
            goal.pose.position.y = 13

    if model_index==0:#hold
        print('READY')
        goal.pose.position.x =ugv1_state_x
        goal.pose.position.y =ugv1_state_y
    if model_index==10:#BLUE_PURSUIT
        print('End')
        goal.pose.position.x =ugv1_state_x
        goal.pose.position.y =ugv1_state_y

           
    # if model_index==13:# LURE_DEEP
    #     goal.pose.position.x =11
    #     goal.pose.position.y = 24
        

    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0

    goal.pose.orientation.y = 0.0

    goal.pose.orientation.z = 0.0

    goal.pose.orientation.w = 1.0


    goal_pub.publish(goal)

    rate.sleep()

ugv0_state_x=0
ugv0_state_y=0
ugv0_state_z=0
ugv1_state_x=0
ugv1_state_y=0
ugv1_state_z=0
blue_uav1_pose = PoseStamped()
blue_uav2_pose = PoseStamped()
red_uav1_pose = PoseStamped()
red_uav2_pose = PoseStamped()
def ugv0_state_callback(data):
    global ugv0_state_x
    global ugv0_state_y
    global ugv0_state_z
    ugv0_state_x=data.data[0]
    ugv0_state_y=data.data[1]
    ugv0_state_z=data.data[2]
  
def ugv1_state_callback(data):
    global ugv1_state_x
    global ugv1_state_y
    global ugv1_state_z 
    ugv1_state_x=data.data[0]
    ugv1_state_y=data.data[1]
    ugv1_state_z=data.data[2]
def blue_uav1_pose_callback(data):
    global blue_uav1_pose
    blue_uav1_pose=data
def blue_uav2_pose_callback(data):
    global blue_uav2_pose
    blue_uav2_pose=data
def red_uav1_pose_callback(data):
    global red_uav1_pose
    red_uav1_pose=data
def red_uav2_pose_callback(data):
    global red_uav2_pose
    red_uav2_pose=data
def state_machine_callback(data):
    global model_index
    model_index=data.mode.mode
    print(model_index)

if __name__ == '__main__':

    try:
        rospy.Subscriber('/ugv0/curr_state', Float32MultiArray, ugv0_state_callback)
        rospy.Subscriber('/ugv1/curr_state', Float32MultiArray, ugv1_state_callback)
        # pub_pose = rospy.Publisher('/blue/ugv1/state_in_map/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/blue/uav1/state_in_map/pose', PoseStamped, blue_uav1_pose_callback)
        rospy.Subscriber('/blue/uav2/state_in_map/pose', PoseStamped, blue_uav2_pose_callback)
        rospy.Subscriber('/red/uav1/state_in_map/pose', PoseStamped, red_uav1_pose_callback)
        rospy.Subscriber('/red/uav2/state_in_map/pose', PoseStamped,  red_uav2_pose_callback)
        # rospy.Subscriber('/blue/uav1/health_value', Temperature, health_callback_uav1)
        # rospy.Subscriber('/blue/uav2/health_value', Temperature, health_callback_uav2)
        rospy.Subscriber('/blue/mode', BlueMode, state_machine_callback)
        while not rospy.is_shutdown():

            # print("Enter 'y' for goal 1, 'u' for goal 2 ")
            # pose_subscriber

            send_goal()
            # rospy.spin()
            pose_msg = PoseStamped()

    except rospy.ROSInterruptException:

        pass
