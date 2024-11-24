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

import serial
from std_msgs.msg import UInt16

from AndroidControllerReceiver import AndroidControllerComm
from serial_interface import serialInterface
from file_handler import fileHandler
redFlag=False
greenFlag=False
yellowFlag=True
"""###############USE FOR RECEIVE ANDROID CONTROL ADDED BY ZMY###################"""

def receive_info_callback(info):
    global redFlag, greenFlag, yellowFlag
    info=info.decode("utf-8")
    print(f"receive:{info}")
    redFlag=False
    greenFlag=False
    yellowFlag=False
    if "red" == info:
        redFlag=True
    if "green" == info:
        greenFlag=True
    if "yellow" == info:
        yellowFlag=True
    if "sound" == info:        
        redFlag=False
        greenFlag=False
        yellowFlag=False

print('1')
accomm=AndroidControllerComm()
print('2')
accomm.androidcontroller_receive_info(receive_info_callback)
print('3')

"""###############USE FOR RECEIVE ANDROID CONTROL END###################"""

class serialInterface(object):
    def __init__(self, port, baud=115200, debug=False):
        # self.arduino = serial.Serial(port, baud)
        self.arduino =serial.Serial(port, baud, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        # self.size = 10
        # self.packet = [0] * self.size
        # self.debug = debug
        # self.struct = None
        # self.state_update_rate=rate
        if self.arduino.isOpen():
            print('open_succ')

    def write(self, data):
        self.arduino.write(data)

    def update_state(self):
        # read data from uart and publish it on respective topic
        rate = rospy.Rate(1)
        # initial_time = timer()
        # self.arduino.st
        while not rospy.is_shutdown():
            if(redFlag):
                # msg = UInt16MultiArray()
                # msg.data = [0x11]
                # msg = b'\x11'
                hex_data=0x11
                self.write(hex_data.to_bytes(1,byteorder='big'))
                # self.write(0x11)
                # self.write(bytes.fromhex('11'))

                print('Red_on')
            else:
                # msg = UInt16MultiArray()
                # msg.data = [0x21]
                # msg = b'\x21'
                # self.write(0x21)
                hex_data=0x21
                self.write(hex_data.to_bytes(1,byteorder='big'))
                print('Red_off')
            if(greenFlag):
                # msg = UInt16MultiArray()
                # msg.data = [0x14]
                # msg = b'\x14'
                # self.write(0x14)
                hex_data=0x14
                self.write(hex_data.to_bytes(1,byteorder='big'))
                print('Rreen_on')
            else:
                # msg = UInt16MultiArray()
                # msg.data = [0x24]
                # msg = b'\x24'
                hex_data=0x24
                self.write(hex_data.to_bytes(1,byteorder='big'))
                # self.write(0x24)
                print('green_off')
            if(yellowFlag):
                # msg = UInt16MultiArray()
                # msg.data = [0x12]
                # msg = b'\x12'
                hex_data=0x12
                print(hex_data.to_bytes(1,byteorder='big'))
                # print(0x14)
                # self.write(b'\x14')
                self.write(hex_data.to_bytes(1,byteorder='big'))
                print(hex_data.to_bytes(1,byteorder='big'))
                print('yellow_on')
            else:
                # msg = UInt16MultiArray()
                # msg.data = [0x22]
                # msg = b'\x22'
                hex_data=0x22
                # print(hex_data.to_bytes(1,byteorder='big'))
                self.write(hex_data.to_bytes(1,byteorder='big'))

                print('yellow_off')
                # self.data1_pub.publish(current_ma_1)
            # break
            rate.sleep()



# class DRV10987Interface:
#     def __init__(self, port, baud, folder="dummy_data/", test_name='zph', test_specs= None):
#         # Only argument stuff
#         self.running = False
#         self.test_specs = test_specs
#         self.serial_trx = serialInterface(port, baud, debug = True)
#         self.file_handler = fileHandler(folder, test_name)
#         self.cmd_code = 0
#         self.speed = 0

#     def initialize(self):
#         # Get params and allocate msgs
#         self.state_update_rate = rospy.get_param('/rate', 1000)
#         self.serial_trx.initialize()
#         # self.file_handler.init()

#     def start(self):
#         # Create subs, services, publishers, threads
#         self.running = True
#         #publishers
#         # self.data1_pub = rospy.Publisher('/light', UInt16MultiArray, queue_size=70)
#         Thread(target=self.update_state).start()
#         #subscribers
#         # self.cmd_sub = rospy.Subscriber('/light', UInt16MultiArray, self.cmd)

#     def stop(self):
#         self.running = False
#         # self.data1_pub.unregister()
#         # self.cmd_sub.unregister()

#     def cmd(self, msg):
#         # get input from /cmd topic and send it by uart via serialInterface
#         self.cmd_code = msg.data[0] #motor id
#         self.speed = msg.data[1] #motor speed
#         self.serial_trx.write(self.cmd_code, self.speed)
#         print('msg')
#         print(msg)
#         print('redFlag')
#         print(redFlag)
#         print('greenFlag')
#         print(greenFlag)
#         print('yellowFlag')
#         print(yellowFlag)

#     def update_state(self):
#         # read data from uart and publish it on respective topic
#         rate = rospy.Rate(self.state_update_rate)
#         initial_time = timer()
#         while self.running and not rospy.is_shutdown():
#             if(redFlag):
#                 msg = UInt16MultiArray()
#                 msg.data = [0x11]
#                 self.cmd(msg)
#             else:
#                 msg = UInt16MultiArray()
#                 msg.data = [0x21]
#                 self.cmd(msg)
#             if(greenFlag):
#                 msg = UInt16MultiArray()
#                 msg.data = [0x14]
#                 self.cmd(msg)
#             else:
#                 msg = UInt16MultiArray()
#                 msg.data = [0x24]
#                 self.cmd(msg)
#             if(yellowFlag):
#                 msg = UInt16MultiArray()
#                 msg.data = [0x12]
#                 self.cmd(msg)
#             else:
#                 msg = UInt16MultiArray()
#                 msg.data = [0x22]
#                 self.cmd(msg)
#                 # self.data1_pub.publish(current_ma_1)
#             rate.sleep()

# def uart_sender():
#     rospy.init_node('uart_sender', anonymous=True)
#     port = '/dev/ttyUSB0'
#     baud = 9600  
#     serial_conn = serialInterface(port, baud, debug=True)

    # def callback(data):
    #     # rospy.loginfo(f"Received data to send: {data.data}")
    #     byte_to_send = data.data.to_bytes(1, byteorder='big')
    #     serial_conn.write(byte_to_send)
    #     rospy.loginfo(f"Sent {byte_to_send} to UART")

    # rospy.Subscriber('uart_send', UInt16, callback)
    # rospy.loginfo("UART sender node started. Waiting for data...")
    # rospy.spin()

if __name__ == '__main__':
    # Sirial comunication parameters
    port = '/dev/ttyUSB0'
    baud = 9600
    # Test Information
    rospy.init_node('ros_uart_interface')
    serial_conn = serialInterface(port, baud, debug=True)
    print('ready')
    serial_conn.update_state()
    # Start test
    # print("Starting DRV10987 ROS-UART Controller")
    # controller = DRV10987Interface(port, baud)
    # controller.initialize()
    # controller.start()
    # rospy.spin()
    # controller.stop()
