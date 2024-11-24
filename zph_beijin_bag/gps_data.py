#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

def convert_to_decimal(degree_minute, direction):

    degrees = int(degree_minute[:-7])
    minutes = float(degree_minute[-7:])
    decimal = degrees + minutes / 60.0
    if direction == 'S' or direction == 'W':
        decimal = -decimal
    return decimal

def parse_nmea_sentence(nmea_sentence):
    data = nmea_sentence.split(',')
    # print('data[0]')
    # print(data[0])
    # print('data[1]')
    # print(data[1])
    # print('data[2]')
    # print(data[2])

    if data[0] == "$GNGLL":
        # Example for GPS Fix data
        # data=['']
        if data[1]=='':
            print('data')
            print(data)
            return  None, None      
        print('data')
        print(data)
        print('data[0]')
        print(data[0])
        print('data[1]')
        print(data[1])
        print('data[2]')
        print(data[2])
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = rospy.Time.now()
        navsatfix_msg.header.frame_id = "gps"
        # Parse latitude and longitude from NMEA sentence
        # You'll need to implement proper parsing and conversion here
        navsatfix_msg.latitude = convert_to_decimal(data[3], data[4])#float(data[3])
        navsatfix_msg.longitude = convert_to_decimal(data[5], data[6])#float(data[5])
        navsatfix_msg.altitude = convert_to_decimal(data[7], data[8]) #float(data[1])  # Altitude data not available in GNRMC
        print('navsatfix_msg.latitude')
        print(navsatfix_msg.latitude)
        print('navsatfix_msg.longitude')
        print(navsatfix_msg.longitude)
        print('navsatfix_msg.altitude')
        print(navsatfix_msg.altitude)
        return navsatfix_msg, None
    elif data[0] == "$GNVTG":
        # Example for heading data
        heading_msg = String()
        heading_msg.data = "Heading: " + data[1] + " degrees"
        return None, heading_msg
    else:
        return None, None

def gps_publisher():
    rospy.init_node('gps_publisher', anonymous=True)
    gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    heading_pub = rospy.Publisher('gps/heading', String, queue_size=10)

    port = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 9600)
    ser = serial.Serial(port, baudrate, timeout=1)

    rospy.loginfo("Starting GPS publisher node...")

    while not rospy.is_shutdown():
        line = ser.readline().decode('ascii', errors='replace').strip()
        navsatfix_msg, heading_msg = parse_nmea_sentence(line)
        if navsatfix_msg:
            gps_pub.publish(navsatfix_msg)
        if heading_msg:
            heading_pub.publish(heading_msg)

    ser.close()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
