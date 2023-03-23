#include "headers.h"
#include "Functions.h"
#include "var.h"

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

void loop()
{
  nh.loginfo("Encoder Value");
  encoder.data = currentPosition;
  ENC_Value.publish( &encoder );
  nh.spinOnce();
  delay(100);
}

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
//Function that controls the variables: 
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
