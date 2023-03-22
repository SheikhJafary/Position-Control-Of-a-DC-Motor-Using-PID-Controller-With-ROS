ros::NodeHandle  nh;
std_msgs::Int64 encoder; 
ros::Publisher ENC_Value("ENC_Value", &encoder);
ros::Subscriber<std_msgs::Int16> PWM_Value("PWM_Value", &get_apply_pwm);


#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8
#define MOTOR_PWM_PIN 5


const int encoderPinA = 2;
const int encoderPinB = 3;

volatile int64_t currentPosition = 0;


// arduino hardware pins
#define MOTOR 0
#define BRAKE 0
#define CW    1
#define CCW   2
