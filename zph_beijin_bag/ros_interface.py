'''
* @file ros_interface.py
* @author Gustavo Diaz H.
* @date 04 Aug 2020
* @brief ROS node interface for communication with
*        DRV10987 BLDC driver Arduino Firmware
'''

import rospy
from threading import Thread
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt16MultiArray
from timeit import default_timer as timer

from ros_uart_controller.AndroidControllerReceiver import AndroidControllerComm
from serial_interface import serialInterface
from file_handler import fileHandler
redFlag=False
greenFlag=False
yellowFlag=False
"""###############USE FOR RECEIVE ANDROID CONTROL ADDED BY ZMY###################"""
"添加到程序启动函数中"
def receive_info_callback(info):
    global redFlag, greenFlag, yellowFlag
    info=info.decode("utf-8")
    # print(f"receive:{info}")
    redFlag=False
    greenFlag=False
    yellowFlag=False
    if "red" == info:
        redFlag=True
    if "green" == info:
        greenFlag=True
    if "yellow" == info:
        yellowFlag=True
accomm=AndroidControllerComm()
accomm.androidcontroller_receive_info(receive_info_callback)

"""###############USE FOR RECEIVE ANDROID CONTROL END###################"""
class DRV10987Interface:
    def __init__(self, port, baud, folder="dummy_data/", test_name='zph', test_specs= None):
        # Only argument stuff
        self.running = False
        self.test_specs = test_specs
        self.serial_trx = serialInterface(port, baud, debug = True)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_code = 0
        self.speed = 0

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 1000)
        self.serial_trx.initialize()
        self.file_handler.init()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #publishers
        # self.data1_pub = rospy.Publisher('/light', UInt16MultiArray, queue_size=70)
        Thread(target=self.update_state).start()
        #subscribers
        # self.cmd_sub = rospy.Subscriber('/light', UInt16MultiArray, self.cmd)

    def stop(self):
        self.running = False
        # self.data1_pub.unregister()
        # self.cmd_sub.unregister()

    def cmd(self, msg):
        # get input from /cmd topic and send it by uart via serialInterface
        self.cmd_code = msg.data[0] #motor id
        self.speed = msg.data[1] #motor speed
        self.serial_trx.write(self.cmd_code, self.speed)
        print('msg')
        print(msg)
        print('redFlag')
        print(redFlag)
        print('greenFlag')
        print(greenFlag)
        print('yellowFlag')
        print(yellowFlag)

    def update_state(self):
        # read data from uart and publish it on respective topic
        rate = rospy.Rate(self.state_update_rate)
        initial_time = timer()
        while self.running and not rospy.is_shutdown():
            if(redFlag):
                self.cmd([0x11])
            else:
                self.cmd([0x21])
            if(greenFlag):
                self.cmd([0x14])
            else:
                self.cmd([0x24])
            if(yellowFlag):
                self.cmd([0x12])
            else:
                self.cmd([0x22])
                # self.data1_pub.publish(current_ma_1)
            rate.sleep()

if __name__ == '__main__':
    # Sirial comunication parameters
    port = '/dev/ttyACM0'
    baud = 9600
    # Test Information
    rospy.init_node('ros_uart_interface')
    test_name = input("test_name: ")
    # Start test
    print("Starting DRV10987 ROS-UART Controller")
    controller = DRV10987Interface(port, baud, folder, test_name, test_specs)
    controller.initialize()
    controller.start()
    rospy.spin()
    controller.stop()
