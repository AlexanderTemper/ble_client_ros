#!/usr/bin/python
import pygatt
import rospy
import PID
import time
from sensor_msgs.msg import Joy
from ble_client_msgs.msg import HandleDroneData


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
t_min = 612;
yaw     = 1024
pitch   = 1024
roll    = 1024
arm = 0
acro = 0
watchdog = 0;
watchdog_old = 0;
hold = -1;
hover_thrust = 860;

pid = PID.PID(0.2, 0.1, 0.01)
pid.setWindup(150)


def regler(ist,soll):
    pid.update(ist)
    t = hover_thrust + pid.output
    # Save window
    if t < t_min:
        return t_min # don't want that rotor stop spinning
    elif t >t_max:
        return t_max # too much
    else:
        return t
    
    

def joy_callback(data):
    global yaw,pitch,roll,thrust, arm, acro, watchdog, hold, pid, hover_thrust

    watchdog = data.header.seq;
    yaw = int((((-1*data.axes[0])+1))*1023.5)

    roll  = int((((-1*data.axes[3])+1))*1023.5)  
    pitch = int((data.axes[4]+1)*1023.5);
    arm   = data.buttons[4]*2047
    acro  = data.buttons[5]*2047
    
    if acro > 1000 and hold <0:
        hold = 400#tofSensor
        pid.SetPoint = 400#hold
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
    global framesend
    try: 
        device.char_write_handle(0x0014, [0x00FF & thrust, thrust >> 8, 0x00FF & roll, roll >> 8, 0x00FF & pitch, pitch >> 8, 0x00FF & yaw, yaw >> 8, 0x00FF & arm, arm >> 8, 0x00FF & acro, acro >> 8]) 
        framesend = framesend + 1
        return True
    except pygatt.exceptions.NotConnectedError:
        return False;

telPitch =0
telRoll =0
telYaw =0
telBat =0
tofSensor = 0
timeused = totaltime = framemissed = frametotal = framesuccess = 0
framesend = 0;
timer = 0;
def handle_data(handle, value):
    global telPitch,telRoll,telYaw,telBat,tofSensor, timeused, totaltime, framemissed, framesend, framesuccess
    """
    handle -- integer, characteristic read handle the data was received on
    value -- bytearray, the data returned in the notification
    """

    if value[0]==0x24 and value[1]==0x54: #start $T
        if value[2]==0x41 and len(value)==10:# A
            telPitch = twos_comp(value[3]+(value[4]<<8))
            telRoll  = twos_comp(value[5]+(value[6]<<8))
            telYaw   = twos_comp(value[7]+(value[8]<<8))
        if value[2]==0x53 and len(value)==11:# S
            telBat = twos_comp(value[3]+(value[4]<<8))
            #telCurr  = twos_comp(value[5]+(value[6]<<8))
            #telRSSI   = value[7]
            #telAirSpeed   = value[8]
            #telFlightMode   = value[9]
        if value[2]==0x4C and len(value)==5:# Costum L Frame
            tofSensor = value[3]+(value[4]<<8)
        if value[2]==0x5A and len(value)==13:# Costum Z Frame
            timeused = value[3]+(value[4]<<8)+(value[5]<<16)+(value[6]<<24)
            totaltime = value[7]+(value[8]<<8)+(value[9]<<16)+(value[10]<<24)
            framemissed = value[11]
            framesuccess = value[12]
            
        #else:
           #rospy.loginfo("%d %s",len(value), binascii.hexlify(value))
    #rospy.loginfo("%d %s",len(value), binascii.hexlify(value))          
def dashboard(publisher):
    global timer, framesend
    publisher.publish((float(telBat)/1000), [telPitch, telRoll, telYaw], tofSensor, [float(timeused)/1000, float(totaltime)/1000],float(timeused)/float(11000)*100,[framemissed,framesuccess,framesend])
    if time.time()-timer >= 1:
        framesend = 0;
        timer = time.time()  
 
def twos_comp(val):
    """compute the 2's complement of int value val"""
    if (val & 0x8000): # if sign bit is set e.g., 8bit: 128-255
        val = val - (0x10000)        # compute negative value
    return val                         # return positive value as is


def main(): 
    global watchdog_old,framesend, timer
    try:
        rospy.init_node('droneControl')
        rospy.loginfo("Starting adapter");
        adapter.start()
        rospy.Subscriber('/joy', Joy, joy_callback)
        rate = rospy.Rate(1000/11) # 11ms


        pub = rospy.Publisher('droneDashboard', HandleDroneData,queue_size=10)
        connected = False
        subscribed = False
        
        timer = time.time();
        
        while not rospy.is_shutdown():
            dashboard(pub)
            if not connected:
                rospy.loginfo("Connection faild ")
                device = connect()
                connected = True if device is not None else False
                subscribed = False

            if connected and not subscribed:
                device.subscribe("2d30c082-f39f-4ce6-923f-3484ea480596",callback=handle_data)
                subscribed = True
                
            if watchdog_old < watchdog: # only send Data if we get new Data From Joy
                watchdog_old = watchdog
                if connected:
                    connected = sendControlData(device)
            rate.sleep()
            
            
            
    finally:
        adapter.stop()

if __name__ == '__main__':
    main()
