#!/usr/bin/env python3
#coding:utf-8
import rospy
import sys
import tty
import termios
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray,Float32
from scipy.spatial.transform import Rotation as R
from  local_planner.msg import BlueMode
from  local_planner.msg import BlueModeDef

from  local_planner.msg import RedModeDef
from  local_planner.msg import RedMode

model_index=11

 
def euler2quaternion(euler):
    r = R.from_euler('xyz', euler, degrees=True)
    quaternion = r.as_quat()
    return quaternion

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


# def ugv0_state_callback(data):
#     # rospy.loginfo("Received Pose: %s", data)
#     return data


def send_goal():

   

    goal_pub = rospy.Publisher('/goal_zph_ugv0', PoseStamped, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    goal = PoseStamped()

    goal.header.stamp = rospy.Time.now()

    goal.header.frame_id = "map"

    global model_index
    if model_index==11:#hold
        print('HOLD')
        goal.pose.position.x =0
        goal.pose.position.y =15
    if model_index==12:#BLUE_PURSUIT
        print('BGLUPERSULT')
        goal.pose.position.x =blue_uav2_pose. pose.position.x
        goal.pose.position.y =blue_uav2_pose.pose.position.y
        print(goal.pose.position.x)
        print(goal.pose.position.y)
    if model_index==13:# LURE_DEEP
        print('luredeep')
        goal.pose.position.x =0
        goal.pose.position.y = 10
    if model_index==0:#hold
        print('READY')
        goal.pose.position.x =0#ugv0_state_x
        goal.pose.position.y =0#ugv0_state_y
    if model_index==10:#BLUE_PURSUIT
        print('End')
        goal.pose.position.x =ugv0_state_x
        goal.pose.position.y =ugv0_state_y

           
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
    print('pinlishugv0')
    global ugv0_state_x
    global ugv0_state_y
    global ugv0_state_z
    ugv0_state_x=data.data[0]
    ugv0_state_y=data.data[1]
    ugv0_state_z=data.data[2]
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x=data.data[0]+0.5#+1.4
    pose_msg.pose.position.y=data.data[1]+0.5#+2
    pose_msg.pose.position.z=data.data[2]
    euler=[data.data[0],data.data[1],data.data[2]]
    qua=euler2quaternion(euler)
    pose_msg.pose.orientation.x=qua[0]
    pose_msg.pose.orientation.y=qua[1]
    pose_msg.pose.orientation.z=qua[2]
    pose_msg.pose.orientation.w=qua[3]
    ugv1_pub_pose.publish(pose_msg)
  
def ugv1_state_callback(data):
    print('pinlishugv1')
    global ugv1_state_x
    global ugv1_state_y
    global ugv1_state_z 
    ugv1_state_x=data.data[0]
    ugv1_state_y=data.data[1]
    ugv1_state_z=data.data[2]
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x=data.data[0]+0.5#+1.4
    pose_msg.pose.position.y=data.data[1]+0.5#+2
    pose_msg.pose.position.z=data.data[2]
    euler=[data.data[0],data.data[1],data.data[2]]
    qua=euler2quaternion(euler)
    pose_msg.pose.orientation.x=qua[0]
    pose_msg.pose.orientation.y=qua[1]
    pose_msg.pose.orientation.z=qua[2]
    pose_msg.pose.orientation.w=qua[3]
    ugv2_pub_pose.publish(pose_msg)

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
        rospy.init_node('state_machine', anonymous=True)
        rospy.Subscriber('/ugv0/curr_state', Float32MultiArray, ugv0_state_callback)
        rospy.Subscriber('/ugv1/curr_state', Float32MultiArray, ugv1_state_callback)
        ugv1_pub_pose = rospy.Publisher('/blue/ugv1/state_in_map/pose', PoseStamped, queue_size=10)
        ugv2_pub_pose = rospy.Publisher('/blue/ugv2/state_in_map/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/blue/uav1/state_in_map/pose', PoseStamped, blue_uav1_pose_callback)
        rospy.Subscriber('/blue/uav2/state_in_map/pose', PoseStamped, blue_uav2_pose_callback)
        rospy.Subscriber('/red/uav1/state_in_map/pose', PoseStamped, red_uav1_pose_callback)
        rospy.Subscriber('/red/uav2/state_in_map/pose', PoseStamped,  red_uav2_pose_callback)
        # rospy.Subscriber('/blue/ugv1/health_value', Temperature, health_callback)
        # rospy.Subscriber('/red/uav2/health_value', Temperature, health_callback)
        rospy.Subscriber('/blue/mode', BlueMode, state_machine_callback)
        while not rospy.is_shutdown():

            # print("Enter 'y' for goal 1, 'u' for goal 2 ")
            # pose_subscriber
            print('piv0')
            send_goal()
            # rospy.spin()
            

    except rospy.ROSInterruptException:

        pass
