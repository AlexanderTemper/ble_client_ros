#!/usr/bin/python

import pygatt
import rospy
from sensor_msgs.msg import Joy


# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben


ADDRESS_TYPE   = pygatt.BLEAddressType.random

adapter = pygatt.GATTToolBackend()

device  = 0
thrust  = 0
yaw     = 1024
pitch   = 1024
roll    = 1024
arm = 0
acro = 0
watchdog = 0;
watchdog_old = 0;


def calcbytes():
    global thrustH, thrustL, yawH, yawL, pitchH, pitchL, rollH, rollL, armH, armL, acroH, acroL
    
    thrustH = thrust >> 8
    thrustL = 0x00FF & thrust 

    yawH = yaw >> 8
    yawL = 0x00FF & yaw

    pitchH = pitch >> 8
    pitchL = 0x00FF & pitch

    rollH = roll >> 8
    rollL = 0x00FF & roll

    armH = arm >> 8
    armL = 0x00FF & arm

    acroH = acro >> 8
    acroL = 0x00FF & acro


def joy_callback(data):
    global yaw,pitch,roll,thrust, arm, acro, watchdog

    watchdog = data.header.seq;
    yaw = int((((-1*data.axes[0])+1))*1023.5)

    t = ((1-data.axes[5])/2)
    thrust = 0 if (t == 0) else int(t*512+512)
    roll  = int((((-1*data.axes[3])+1))*1023.5)  
    pitch = int((data.axes[4]+1)*1023.5);
    arm   = data.buttons[4]*2047
    acro  = data.buttons[5]*2047


def connect():
    DEVICE_ADDRESS = rospy.get_param('~device')  
    rospy.loginfo("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        rospy.loginfo("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None;

def sendControlData(device):
    try: 
        calcbytes()
        device.char_write_handle(0x0014, [thrustL, thrustH, rollL, rollH, pitchL, pitchH, yawL, yawH, armL, armH, acroL, acroH]) 
        # rospy.loginfo("%d %d %d %d %d %d %d %d %d %d %d %d", thrustH, thrustL, yawH, yawL, pitchH, pitchL, rollH, rollL, armH, armL, acroH, acroL)
        return True
    except pygatt.exceptions.NotConnectedError:
        return False;

telPitch =0
telRoll =0
telYaw =0
telBat =0
def handle_data(handle, value):
    global telPitch,telRoll,telYaw,telBat
    """
    handle -- integer, characteristic read handle the data was received on
    value -- bytearray, the data returned in the notification
    """

    if value[0]==0x24 and value[1]==0x54: #start $T
        if value[2]==0x41 and len(value)==10:# A
            telPitch = twos_comp(value[3]+(value[4]<<8))
            telRoll  = twos_comp(value[5]+(value[6]<<8))
            telYaw   = twos_comp(value[7]+(value[8]<<8))
            #rospy.loginfo("A FRAME: [%d;%d;%d] crc:%d]", telPitch, telRoll, telYaw,value[9])
        if value[2]==0x53 and len(value)==11:# S
            telBat = twos_comp(value[3]+(value[4]<<8))
            #telCurr  = twos_comp(value[5]+(value[6]<<8))
            #telRSSI   = value[7]
            #telAirSpeed   = value[8]
            #telFlightMode   = value[9]
            #rospy.loginfo("S FRAME: BAT:%d MODE:%s crc:%d", telBat, value[9], value[10] )
        #else:
           #rospy.loginfo("%d %s",len(value), binascii.hexlify(value))
    rospy.loginfo("%.3f [%3d %3d %3d]",(float(telBat)/1000), telPitch, telRoll, telYaw)          
           
 
def twos_comp(val):
    """compute the 2's complement of int value val"""
    if (val & 0x8000): # if sign bit is set e.g., 8bit: 128-255
        val = val - (0x10000)# compute negative value
    return val# return positive value as is


def main(): 
    global watchdog_old
    try:
        rospy.init_node('droneControl')
        rospy.loginfo("Starting adapter");
        adapter.start()
        rospy.Subscriber('/joy', Joy, joy_callback)
        rate = rospy.Rate(90,9090909090) # 11ms

        connected = 0;
        while not rospy.is_shutdown():
            
            if not connected:
                rospy.loginfo("Connection faild ")
                device = connect()
                connected = True if device is not None else False

            if connected:
                device.subscribe("2d30c082-f39f-4ce6-923f-3484ea480596",callback=handle_data)
            if watchdog_old < watchdog: # only send Data if we get new Data From Joy
                watchdog_old = watchdog
                if connected:
                    connected = sendControlData(device)
                

            rate.sleep()
    finally:
        adapter.stop()

if __name__ == '__main__':
    main()
