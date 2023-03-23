Title: Position Control Of a DC Motor Using PID Controller In ROS
==
- [Title: Position Control Of a DC Motor Using PID Controller With ROS](#title-PID-control-of-a-dc-motor-with-ros)
  - [Introduction](#introduction)
  - [Components Used](#components-used)
  - [Programming Languages Used](#programming-languages-used)
  - [Theoretical Description](#theoretical-description)
      - [How does a encoder work?](#how-does-a-encoder-work)
      - [What is a PID Controller?](#what-is-a-pid-controller)
  - [Program Explaination](#program-explaination)
    - [Arduino Code (serial node)](#arduino-code-serial-node)
      - [Code to rotate the motor.](#code-to-rotate-the-motor)
      - [Code for reading the encoder values](#code-for-reading-the-encoder-values)
      - [Circuit Diagram](#circuit-diagram)
    - [Python Node (PID_node.py)](#python-node-PID_node)
  
## Introduction

This project is designed with the purpose of controlling the position of a dc motor with ROS code and it consists of two prats. 
- The motor is controlled using a PID Controller implemented a ROS Node. 
- Encoder Sensing and Motor movement was done using Arduino Uno. 
These two part of the project communicate with each other through the rosserial library/packag.

## Components Used
1. ROS Melodic along with rosserial library/package.
2. Arduino Uno
3. DC Geared Motor with Encoder ( 12v, 300RPM at 10Kgcm RMCS-5033 ) <br />
datasheet :- <a href="/Datasheets/RMCS%205033.pdf" > RMCS 205033  <a/>
4. Motor Shield : VNH3ASP30 DC Motor Driver 2x14A <br />
datasheet :- <a href="/Datasheets/vnh2sp30.pdf" > VNH3ASP30  <a/>
5. 12V 4A AC/DC Adapter

## Programming Languages Used
1. Python
2. C++

## Theoretical Description

There is a Arduino Sketch as shown in this repository where ros.h library is included to enable ros capabilities. Now we use the rosserial package which enables serial communication of Arduino with ROS Environment. Now there is a Node that has a PID Controller implementation. 
Basic overall structure is

Node_motor.py :- 
* Publishes  - PWM_Value
* Subscribes - ENC_Value, desiredPosition

Arduino Node :- 
* Publishes  - ENC_Value
* Subscribes - PWM_Value



Below is the **rqt_graph** of the program

<img src="/Pictures/graph.jpg" alt="encoder" height=90 width=1116/>

Basic Flow of the program :-

1. The encodervalue is published from Arduino node. 
2. node_motor subscribes to /encoderValue and /desiredPosition.
3. Now we Publish the desired angle/position to /desiredPosition.
4. These inputs are provided to the PID controller implemented in the **node_motor.py** where Setpoint is **/desiredPosition** and feedback is /encodervalue.
5. The PID controller gives PWM_value as output to control the motor direction and speed at a given refresh rate.
6. This PWM value is published to /PWM_value and Arduino node subscribes to the /PWM_value and accordingly give commands to the motor driver.

#### How does a encoder work?

The encoder is a sensor that notifies the driver of the speed and position of the motor. The encoders (position detectors) used in the servo motor can be structurally classified as "incremental encoders" and "absolute encoders". 
Incremental encoders, in their basic form, detect light, magnetic or physical changes on a disc that rotates with the shaft.
Here motor is equiped with a optical encoder which observes the transition of light on a receiver as shown in the picture.

<img src="https://sheikhjafari.pgitic.ir/encoder340.jpg" alt="encoder" height=230 width=230/>

Now, if we use a single channel encoder as shown above there is a disadvantage that we cannot determine the direction of the movement of the motor (i.e. Clockwise or Anti-Clockwise)

To overcome this disadvantage we use quadrature encoder as shown below.

<img src="https://sheikhjafari.pgitic.ir/quadrature encoder.jpg" alt="Encoder" height=270 width=467/>

Now we can calculate the phase difference between the output signals of the encoder and determine the direction as shown.

<img src="/Pictures/rotaryencoderoutput.png" alt="Rotary Encoder Output"/>



#### What is a PID Controller?

A PID controller (proportional–integral–derivative controller) is a closed loop control mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an *error value e(t)* as the difference between a desired *setpoint (SP)* and a measured *process variable (PV)* and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).

In our case:-

<img src="https://user-images.githubusercontent.com/56308805/110928110-81996900-834c-11eb-9b04-2e3beeb0f9f1.jpg" alt="pid"/>


We need to tune the P,I,D parameters individually for a specific control system according to the requirement.

<img src="https://user-images.githubusercontent.com/56308805/110928121-852cf000-834c-11eb-933b-8ad6dd6d8df7.png" alt="pid_tunning"/>


Following are general types of system response depending upon the P,I,D values.


<img src="https://user-images.githubusercontent.com/56308805/110928132-8827e080-834c-11eb-950e-7fa571261acb.png" width="500" height=400 alt="introduction-to-pid-damped-controller"/>

## Program Explaination


### Arduino Code (serial node)

####Header files
we integrate the ros.h library to enable ROS compatiblity
Header files for including ros and std_msgs package which is to define the data type of that topic.
Refer to headers.h. 
```cpp
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

```
#### Main Loop
Now we write the publisher inside loop to continuosly publish the updated encodervalues to the given topic.

```cpp
void loop()
{
  nh.loginfo("Encoder Value");
  encoder.data = currentPosition;
  ENC_Value.publish( &encoder );
  nh.spinOnce();
  delay(100);
}
```
#### Code for reading the encoder value
In setup function, we set up the encoder pins as interrupt pins, run timer and initialize ROS node.
```cpp
void setup()
{
  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31.37255 [kHz]// set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  
  //---------------------------------------------- Set Encoder pins and interrupts -----------------------------
  pinMode (encoderPinA, INPUT_PULLUP);// sets the digital pin encoderPinA as input
  pinMode (encoderPinB, INPUT_PULLUP);// sets the digital pin encoderPinB as input
  //Trigger the interrupt whenever the pin encoderPinA changes value and call readEncoderA() function 
  attachInterrupt (digitalPinToInterrupt (encoderPinA), readEncoderA, CHANGE);
  //Trigger the interrupt whenever the pin encoderPinB changes value and call readEncoderB() function
  attachInterrupt (digitalPinToInterrupt (encoderPinB), readEncoderB, CHANGE);

  //---------------------------------------------- Setup Ros Node ----------------------------------------------- 
  nh.initNode(); //initializes the ROS node for the process
  nh.advertise(ENC_Value);// create a ros::Publisher which is used to publish on ENC_Value topic
  nh.subscribe(PWM_Value);//Subscribing to pwm topic
}
```
Now we code the Interrupt service routine program to update the encoder value upon the movement of the shaft
```cpp
void readEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    currentPosition++;//Clockwise rotation
  }
  else
  {
    currentPosition--; //Counter_Clockwise Rotation
  }
}

void readEncoderB()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    currentPosition++;//Clockwise rotation
  }
  else
  {
    currentPosition--;//Counter_Clockwise Rotation
  }
}
```
#### Code to rotate the motor
Defining the Motor Pins for motor1 slot on the shield.
Refer to var.h.
```cpp
// Arduino hardware pins
#define MOTOR_A1_PIN  7   //Positive end for motor 
#define MOTOR_B1_PIN  8   //Negative end for motor 
#define MOTOR_PWM_PIN 5   //Speed control signal for motor

const int encoderPinA = 2;
const int encoderPinB = 3;

volatile int64_t currentPosition = 0;

#define BRAKE 0
#define CW    1
#define CCW   2
```
we write the subscriber for the topic /PWM_Value.

```cpp
ros::Subscriber<std_msgs::Int16> PWM_Value("PWM_Value", &get_apply_pwm);
```

Now we want create a function to move the motor according to the sign of PWM value given to it.
```cpp
//This function gets pwm value as input parameter
//Determines the direction of the motor 
//Call Motor_Drive function
void get_apply_pwm( const std_msgs::Int16& pwm_value){
  int PWM_Val =0;
  PWM_Val = pwm_value.data;
  
  if ( PWM_Val > 0 )
  {
  drive_motor(CCW,PWM_Val);
  }
  else
  {
  drive_motor(CW,abs(PWM_Val));
  }
}
```
Now, we define a function to move the motor according to the direction and PWM Signal
```cpp
//This function controls the variables: 
//direction (cw or ccw) ,
//pwm (0 to 255);
void drive_motor(int16_t direct, uint16_t pwm_Value)         
{
 
    if(direct == CW) //Set Direction Clockwise
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW) //Set Direction Anticlockwise
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else //Stop Motor
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(MOTOR_PWM_PIN, pwm_Value); //Set Speed 
   
}
```
---
#### Circuit Diagram
Circuit diagram for interfacing encoder motor with Arduino Mega and VNH2P30 Motor Shield.
<div>
<img src="https://sheikhjafari.pgitic.ir/motor1.jpg" width=377 height=360 alt="circuit_diagram"/>
<img src="https://sheikhjafari.pgitic.ir/motor2-525x360.jpg" width=525 height=360 alt="circuit_diagram"/>
</div>
---
### Python Node (PID_node.py)

This code is written for the angular control of the motor, but it can be modified for the distance/position control according to the wheel diameter.

importing rospy and std_msgs packages. Defining global variables and motor specifications.
```python
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

encoderValue = 0
started = False
started1 = False
desired_Position = 0.0

```
Defining Callback functions for subscribers and a general function with timer for subscribing to two topics */encoderValue* and */desiredPosition*.
```python
pub = rospy.Publisher('/PWM_Values',Int16,queue_size=100)
pub1 = rospy.Publisher('/Current_angle',Int16,queue_size=100)

def sub_encoderValue():
    rospy.init_node('node_motor', anonymous=True)
    rospy.Subscriber('encoderValue',Int64,callback)
    rospy.Subscriber('desiredPosition',Float64,callback1)
    timer = rospy.Timer(rospy.Duration(0.01),timer_callback)
    rospy.spin()
    timer.shutdown()

def callback(data):
    global started,encoderValue
    print "EncoderValue Received",encoderValue
    encoderValue = data.data
    if (not started):
        started = True

def callback1(data):
    global started1,desired_Position
    desired_Position = data.data
    if (not started1):
        started1 = True
```

Defining a callback function to calculate the PID output according to the updated */encoderValue* and */desiredPosition*. After that the we run the sub_encoder() function with runs whole program.
```python
def timer_callback(event):
    global started,started1,pub,encoderValue,PWM_Output,desired_Position,current_wheel_distance,current_angle
    
    if(started1):
        if (started):

            previous_angle=current_angle
            pid = PID(0.022,0.01,2,setpoint=desired_Position)
            pid.output_limits = (-255, 255)
            pid.sample_time = 0.001
            PWM_Output = pid(previous_angle)

            if( 0 < PWM_Output <= 13):
                PWM_Output = PWM_Output + 11.5
            elif (-13 <= PWM_Output < 0):
                PWM_Output = PWM_Output - 11.5
            
            current_angle = encoderValue/24
            
            pub.publish(PWM_Output)
            
            print "Publishing PWM Values",PWM_Output
            print "Current angle",current_angle
            print "Desired Position",desired_Position

if __name__ == '__main__':
    print "Running"
    sub_encoderValue()

```
---
## Output Video

**Steps to run the code**

```shell
roscore
```
check your port number in Arduino IDE.Running the serial node after uploading the code to arduino
```
rosrun rosserial_arduino serial_node.py /dev/tty/ACM0
```
running the motor control node.
```
rosrun pkg_name node_motor.py
```
Now we publish the required output angle to /desiredPosition topic using terminal.
```
rostopic pub /desiredPosition std_msgs/Float64 "data: 0.0"
```


---
