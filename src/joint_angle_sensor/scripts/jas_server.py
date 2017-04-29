#!/usr/bin/python3

import rospy
from joint_angle_sensor.msg import jas
import serial
import argparse
import sys
import can

can.rc['interface'] = 'socketcan_native'
can.rc['channel'] = 'can0'

idMap = {"Bottom0" : 0x50, "Top0" : 0x51, "Bottom1" : 0x53, "Top1" : 0x52}

##
# expect following data
# 0xFF, 0xFF - data header
# 0xXX - CAN ID
# 0xXX, 0xXX - 12b angle
def receiveDataSerial(serialPort, rospy):
    try:
        data = int(serialPort.read())
        if data == 255:
            data = int(serialPort.read())
            if data == 255:
                channelId = int(serialPort.read())
                data = int(serialPort.read())
                angle = data << 8
                data = int(serialPort.read())
                angle += data
                print("Received channelId " + str(channelId) + "; data = " + str(data))
                return (channelId, angle)

        return (-1, -1)
    except:
        return (-1, -1)

def receiveDataCAN(canBus):
    message = canBus.recv()
    if message is not None:
        channelId = message.arbitration_id
        if message.dlc == 2:
            angle = message.data[0] + (message.data[1] << 8)
            return (channelId, angle)

    return (-1, -1)



def publisher(serialPort, canBus, useSerial):
    pub = rospy.Publisher("joint_angle_sensors", jas, queue_size=10)
    rospy.init_node("robolegs")
    jasmsg = jas()
    while not rospy.is_shutdown():
        if useSerial == True:
            jasData = receiveDataSerial(serialPort, rospy)
        else:
            jasData = receiveDataCAN(canBus)

        if jasData[0] == idMap['Bottom0']:
            jasmsg.Bottom0 = jasData[1]
        if jasData[0] == idMap['Top0']:
            jasmsg.Top0 = jasData[1]
        if jasData[0] == idMap['Bottom1']:
            jasmsg.Bottom1 = jasData[1]
        if jasData[0] == idMap['Top1']:
            jasmsg.Top1 = jasData[1]

        if jasData[0] != -1:
            pub.publish(jasmsg)

if __name__ == "__main__":
    try:
        argv = rospy.myargv(argv=sys.argv)
        parser = argparse.ArgumentParser(description='Publisher for joint angle sensors')
        parser.add_argument("--useSerial", dest="useSerial", action="store_true", help="Use serial interface instead of CAN bus", default='false')
        parser.add_argument("--port", dest="port", help="Path to the serial port device", default='/dev/ttyACM0')
        args = parser.parse_args(argv[1:])

        #serialPort = serial.Serial(args.port, 115200, timeout=1)
        if args.useSerial == True:
            serialPort = serial.Serial(args.port, 9600, timeout=1)
            serialPort.close()
            serialPort.parity = serial.PARITY_ODD
            serialPort.open()
            serialPort.close()
            serialPort.parity = serial.PARITY_NONE
            serialPort.close()
            publisher(serialPort, None, True)
        else:
            canBus = can.interface.Bus()
            publisher(None, canBus, False)
    except rospy.ROSInterruptException:
        pass

