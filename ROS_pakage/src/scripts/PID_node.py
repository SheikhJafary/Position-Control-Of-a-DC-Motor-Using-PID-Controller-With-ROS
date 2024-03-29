#!/usr/bin/env python
import rospy
from simple_pid import PID

from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Int16


PWM_Output = 0
PPR = 41
gearRatio = 60
decodeNumber = 4
pi = 3.14
r = 0.0325
ENC_Value = 0
started = False
started1 = False
desired_Position = 0.0

current_wheel_distance = (ENC_Value * 2 * pi *r) / (PPR * gearRatio * decodeNumber)
current_angle = ENC_Value*((PPR * gearRatio * decodeNumber)/360)

pub = rospy.Publisher('/PWM_Value',Int16,queue_size=100)
#pub1 = rospy.Publisher('/Current_angle',Int16,queue_size=100)
def Func_ENC_Value():
    rospy.init_node('NODE_PID', anonymous=True)
    rospy.Subscriber('ENC_Value',Int64,callback)
    rospy.Subscriber('desiredPosition',Float64,callback1)
    timer = rospy.Timer(rospy.Duration(0.01),timer_callback)
    rospy.spin()
    timer.shutdown()

def callback(data):
    global started,ENC_Value
    print "ENC_Value Received",ENC_Value
    ENC_Value = data.data
    if (not started):
        started = True

def callback1(data):
    global started1,desired_Position
    #print "Desired Received",desired_Position
    desired_Position = data.data
    if (not started1):
        started1 = True



def timer_callback(event):
    global started,started1,pub,ENC_Value,PWM_Output,desired_Position,current_wheel_distance,current_angle
   
    if(started1):
        if (started):

            previouswheeldistance = current_wheel_distance
            pid = PID(100,0.5,1, setpoint=desired_Position,auto_mode=True)
            pid.output_limits = (-255, 255)
            pid.sample_time = 0.001
            PWM_Output = pid(previouswheeldistance)
            current_wheel_distance = (ENC_Value * 2 * pi *r) / (PPR * gearRatio * decodeNumber)

           
            pub.publish(PWM_Output)
           
            print "Publishing PWM Values",PWM_Output
            print "Current Wheel distance",current_wheel_distance


if __name__ == '__main__':
    print "Running"
    Func_ENC_Value()

