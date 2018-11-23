#!/usr/bin/python

import pygatt
import rospy
import PID
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
t_max = 1024;
t_min=512;
yaw     = 1024
pitch   = 1024
roll    = 1024
arm = 0
acro = 0
watchdog = 0;
watchdog_old = 0;
hold = -1;

pid = PID.PID(1, 1, 0.001)



def regler(ist,soll):
    pid.update(ist)
    if pid.output < t_min:
        return t_min #don't want that rotor stop spinning
    elif pid.output >t_max:
        return t_max #too much
    else:
        return pid.output
    
    

def joy_callback(data):
    global yaw,pitch,roll,thrust, arm, acro, watchdog, hold, pid

    watchdog = data.header.seq;
    yaw = int((((-1*data.axes[0])+1))*1023.5)

    roll  = int((((-1*data.axes[3])+1))*1023.5)  
    pitch = int((data.axes[4]+1)*1023.5);
    arm   = data.buttons[4]*2047
    acro  = data.buttons[5]*2047
    
    if acro > 1000 and hold <0:
        hold = tofSensor
        pid.SetPoint = hold
    elif acro > 1000 and hold >0:
        thrust = int(regler(tofSensor, hold))
        
    if acro <=1000:
        hold = -1
        t = ((1-data.axes[5])/2)
        thrust = 0 if (t == 0) else int(t*(t_max-t_min)+t_min)
    
    

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
        device.char_write_handle(0x0014, [0x00FF & thrust, thrust >> 8, 0x00FF & roll, roll >> 8, 0x00FF & pitch, pitch >> 8, 0x00FF & yaw, yaw >> 8, 0x00FF & arm, arm >> 8, 0x00FF & acro, acro >> 8]) 
        # rospy.loginfo("%d %d %d %d %d %d %d %d %d %d %d %d", thrustH, thrustL, yawH, yawL, pitchH, pitchL, rollH, rollL, armH, armL, acroH, acroL)
        return True
    except pygatt.exceptions.NotConnectedError:
        return False;

telPitch =0
telRoll =0
telYaw =0
telBat =0
tofSensor = 0;
def handle_data(handle, value):
    global telPitch,telRoll,telYaw,telBat,tofSensor
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
           # telAirSpeed   = value[8]
            #telFlightMode   = value[9]
            #rospy.loginfo("S FRAME: BAT:%d MODE:%s crc:%d", telBat, value[9], value[10] )
        if value[2]==0x4C and len(value)==5:# Costum L Frame
            tofSensor = value[3]+(value[4]<<8)
        #else:
           #rospy.loginfo("%d %s",len(value), binascii.hexlify(value))
    #rospy.loginfo("%d %s",len(value), binascii.hexlify(value))          
           
 
def twos_comp(val):
    """compute the 2's complement of int value val"""
    if (val & 0x8000): # if sign bit is set e.g., 8bit: 128-255
        val = val - (0x10000)        # compute negative value
    return val                         # return positive value as is


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
                

            #rospy.loginfo("%.3f [%3d %3d %3d] %4d",(float(telBat)/1000), telPitch, telRoll, telYaw, tofSensor)
            rospy.loginfo("%4d %4d %d", tofSensor, hold, thrust)
            rate.sleep()
    finally:
        adapter.stop()

if __name__ == '__main__':
    main()
