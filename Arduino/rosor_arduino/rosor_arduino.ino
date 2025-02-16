#include <ArduinoSTL.h>
#include <ros.h>
#include <std_msgs/Float64.h>


#define LEFT_MOTOR_A  11
#define LEFT_MOTOR_B  10
#define RIGHT_MOTOR_A  6
#define RIGHT_MOTOR_B  5
#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3
#define ENCODER_N   20
#define ENCODER_DELAY   20

int left_motor_pwm = 0; 
int right_motor_pwm = 0;

ros::NodeHandle nh;
std_msgs::Float64 leftenc_msg;  
std_msgs::Float64 rightenc_msg;
ros::Publisher left_encoder("/robot/left_wheel/encoder", &leftenc_msg);
ros::Publisher right_encoder("/robot/right_wheel/encoder", &rightenc_msg);  

void left_motor_write(const std_msgs::Float64& msg)
{
  left_motor_pwm = (int)msg.data;
}

void right_motor_write(const std_msgs::Float64& msg)
{
  right_motor_pwm = (int)msg.data;
}

ros::Subscriber<std_msgs::Float64> left_mot_sub("/robot/left_wheel/pwm", left_motor_write);
ros::Subscriber<std_msgs::Float64> right_mot_sub("/robot/right_wheel/pwm", right_motor_write);


unsigned long pre_left_time = millis();
unsigned long pre_right_time = millis();
unsigned long pre_Time = millis();

long left_encoder_count = 0, right_encoder_count = 0;

void drive(int left_pwm, int right_pwm)
{
  if (left_pwm >= 0) {
    digitalWrite(LEFT_MOTOR_A, 1);
    analogWrite(LEFT_MOTOR_B, 255 - left_pwm);
  }
  else {
    left_pwm = -left_pwm;
    digitalWrite(LEFT_MOTOR_B, 1);
    analogWrite(LEFT_MOTOR_A, 255 - left_pwm);
  }

  if (right_pwm >= 0) {
    digitalWrite(RIGHT_MOTOR_A, 1);
    analogWrite(RIGHT_MOTOR_B, 255 - right_pwm);
  }
  else {
    right_pwm = -right_pwm;
    digitalWrite(RIGHT_MOTOR_B, 1);
    analogWrite(RIGHT_MOTOR_A, 255 - right_pwm);
  }
}

void encoder_counter_left()
{
  if (millis() - pre_left_time > ENCODER_DELAY)
  {
    if (left_motor_pwm > 0) 
      left_encoder_count++;
    else if (left_motor_pwm < 0)
      left_encoder_count--;
    pre_left_time = millis();
  }
  //Serial.print("left_encoder: \t");
  //Serial.println(left_encoder);
}

void encoder_counter_right()
{
  if (millis() - pre_right_time > ENCODER_DELAY)
  {
    if (right_motor_pwm > 0)
      right_encoder_count++;
    else if (right_motor_pwm < 0)
      right_encoder_count--;
    pre_right_time = millis();
  }
  //Serial.print("right_encoder: \t");
  //Serial.println(right_encoder);
}



void setup()
{
  nh.initNode();
  nh.advertise(left_encoder); 
  nh.advertise(right_encoder);
  nh.subscribe(left_mot_sub);
  nh.subscribe(right_mot_sub);

  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);

  pinMode(ENCODER_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), encoder_counter_left, RISING);
  pinMode(ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), encoder_counter_right, RISING);
  
  drive(0, 0);
}

void loop()
{
  float left_encoder_rad = (float)left_encoder_count / ENCODER_N * TWO_PI;
  float right_encoder_rad = (float)right_encoder_count / ENCODER_N * TWO_PI;
  
  if (millis() - pre_Time > ENCODER_DELAY)
  {  
    leftenc_msg.data = left_encoder_rad; 
    left_encoder.publish(&leftenc_msg);
    rightenc_msg.data = right_encoder_rad; 
    right_encoder.publish(&rightenc_msg);  
    
    nh.spinOnce();
    pre_Time = millis();
  }
  drive(left_motor_pwm, right_motor_pwm);
}
