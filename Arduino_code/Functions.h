void pwm_input( const std_msgs::Int16& pwm_value);
void get_apply_pwm( const std_msgs::Int16& pwm_value);
void drive_motor(int16_t motor_en, int16_t direct, uint16_t pwm) ;
void readEncoderA();
void readEncoderB();
