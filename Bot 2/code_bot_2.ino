#include "Kinematics.h"
#include "math.h"

#include<LiquidCrystal_I2C.h> 

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <string>
#include <std_msgs/Int32.h>
#include <Servo.h>


ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;
std_msgs::Int32 pulse1_msg, pulse2_msg, pulse3_msg, pulse4_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_shield_data", &imu_msg);


ros::Publisher frontleft_pub("frontleft", &pulse1_msg);
ros::Publisher frontright_pub("frontright", &pulse2_msg);
ros::Publisher rearleft_pub("rearleft", &pulse3_msg);
ros::Publisher rearright_pub("rearright", &pulse4_msg);
/*SampleConfig config;
IMPU& mpu = config.getMPU();
*/
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))


#define MOTOR_MAX_RPM 468      // motor's maximum rpm
#define WHEEL_DIAMETER 0.127      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.540   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.540   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)

LiquidCrystal_I2C lcd(0x27,16,2);

#define ENCA1 29
#define ENCB1 27
#define ENCA2 38
#define ENCB2 40
#define ENCA3 24
#define ENCB3 22
#define ENCA4 30
#define ENCB4 32
#define PWM1 3
#define DIR1 31
#define PWM2 4 
#define DIR2 42 
#define PWM3 11
#define DIR3 41 
#define PWM4 10
#define DIR4 44


#define BDPWM 8 //5belt drive 
#define BDDIR 37
#define FLAPDIR 49 // flapper dc motor
#define FLAPPWM 7
#define FLAPENCCHA 45
#define FLAPENCCHB 47
#define Pi 3.14159265359


#include "Arduino_NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#define Wire Wire1


NineAxesMotion mySensor;         //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;
bool updateSensorData = true;  
float euler_x=0,euler_y=0,euler_z=0,euler_w=0;


int lastTime1=0, lastTime2=0, lastTime3=0, lastTime4=0;
//float Kp1 = 0.439391792581009, Kd1 = 0.0152141737371059, Ki1 = 0.00;//6330970643622;
//float Kp1 = 5, Kd1 = 0.0, Ki1 = 0.01;
float Kp1 = 0.189391792581009, Kd1 = 0.0052141737371059 , Ki1 = 0.005330970643622;


long prevT = 0;
long prevT1 = 0;
float deltaT1=0;


float linear_vel_x=0; // Velocities
float linear_vel_y=0;
float angular_vel_z=0;


float rpm1,rpm2,rpm3,rpm4; // Input RPM from the motor
volatile int pulse1 = 0, pulse2 = 0, pulse3 = 0, pulse4 = 0; // Pulse value from the encoder
float curr_rpm1=0,curr_rpm2=0,curr_rpm3=0,curr_rpm4=0; // RPM values calculated from encoder
float eprev = 0, eintegral = 0; //  PID control
int x_pos=0, y_pos=0, heading=0; // Position of the Robot
volatile int prev_pulse1=0,prev_pulse2=0,prev_pulse3=0,prev_pulse4=0;
float ppr1=136,ppr2=133,ppr3=136,ppr4=128;


volatile long bdencoderCount[2] = {0, 0};
int ros_pb_var = 0, ros_pb_var1 = 0;
int clr_ball = 4;
int r1 = 1, r2 = 0;


volatile long flapencpulse = 0;


Servo esc1; // Create a Servo object
Servo esc2; // Create a Servo object


void velocityCallback(const geometry_msgs::Twist& msg) {
   twist_msg = msg;
   linear_vel_x = twist_msg.linear.x;
   linear_vel_y = twist_msg.linear.y;
   angular_vel_z = twist_msg.angular.z;
}

int PIDcompute(int rpm,int curr_rpm); // function declaration


// Callback functions for the ROS subscribers
void twoIntsCallbackB(const std_msgs::Int32& msg) {
 clr_ball = msg.data;
}
// int clr_ball = (clr!=4&&clr!=0)?clr:clr_ball;


void oneIntCallback(const std_msgs::Int32& msg) {
 ros_pb_var = msg.data;
}


ros::Subscriber<std_msgs::Int32> sub_b("one_int_colour", twoIntsCallbackB);
ros::Subscriber<std_msgs::Int32> sub_one_int("one_int", oneIntCallback);


//* command velocities from ROS passed to the function***//////
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);


ros::Subscriber<geometry_msgs::Twist> sub_linear_x("cmd_vel", &velocityCallback);
ros::Subscriber<geometry_msgs::Twist> sub_linear_y("cmd_vel", &velocityCallback);
ros::Subscriber<geometry_msgs::Twist> sub_angular_z("cmd_vel", &velocityCallback);


void setup()
{
 Serial.begin(57600);
 lcd.begin();
    // Print a message to the LCD.
  lcd.backlight();
  lcd.setBacklight(HIGH);
 pinMode(ENCA1, INPUT);
 pinMode(ENCB1, INPUT);
 pinMode(ENCA2, INPUT);
 pinMode(ENCB2, INPUT);
 pinMode(ENCA3, INPUT);
 pinMode(ENCB3, INPUT);
 pinMode(ENCA4, INPUT);
 pinMode(ENCB4, INPUT);
 pinMode(PWM1, OUTPUT);
 pinMode(DIR1, OUTPUT);
 pinMode(PWM2, OUTPUT);
 pinMode(DIR2, OUTPUT);
 pinMode(PWM3, OUTPUT);
 pinMode(DIR3, OUTPUT);
 pinMode(PWM4, OUTPUT);
 pinMode(DIR4, OUTPUT);
 attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
 attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
 attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
 attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);


 analogWrite(PWM1,0);
 analogWrite(PWM2,0);
 analogWrite(PWM3,0);
 analogWrite(PWM4,0);


//  for (int i = 0; i < 2; i++) {
//    pinMode(bdencA[i], INPUT);
//    pinMode(bdencB[i], INPUT);
//  }
 pinMode(BDPWM, OUTPUT);
 pinMode(BDDIR, OUTPUT);


 analogWrite(BDPWM, 0);
 esc1.attach(13); // Attach the ESC to pin 13 on the Arduino
 esc2.attach(9); // Attach the ESC to pin 9 on the Arduino
 esc1.writeMicroseconds(1500); // Initialize the motor to a neutral position
 esc2.writeMicroseconds(1500); // Initialize the motor to a neutral position


 pinMode(FLAPDIR, OUTPUT);
 pinMode(FLAPPWM, OUTPUT);
//  pinMode(FLAPENCCHA, INPUT);
//  pinMode(FLAPENCCHB, INPUT);


 analogWrite(FLAPPWM, 0);
//  attachInterrupt(digitalPinToInterrupt(bdencA[0]), readBDEncoder1, RISING);
//  attachInterrupt(digitalPinToInterrupt(bdencA[1]), readBDEncoder2, RISING);
//  for (int i = 0; i < 2; i++) {
//    bdencoderCount[i] = 0;
//  }
 analogWrite(BDPWM, 0);


 Wire.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
 //Sensor Initialization
 mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
 mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
 mySensor.setUpdateMode(MANUAL); //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
 //Setting to MANUAL requires fewer reads to the sensor
 mySensor.updateAccelConfig();
 updateSensorData = true;
 nh.initNode();
 
 nh.subscribe(sub_linear_x); // Subscribe to linear_x topic
 nh.subscribe(sub_linear_y); // Subscribe to linear_y topic
 nh.subscribe(sub_angular_z); // Subscribe to angular_z topic
 nh.advertise(frontleft_pub);
 nh.advertise(frontright_pub);
 nh.advertise(rearleft_pub);
 nh.advertise(rearright_pub);


 nh.advertise(imu_pub);


 imu_msg.header.frame_id = "imu_link";
 imu_msg.orientation_covariance[0] = -1; // Orientation data not available
 imu_msg.angular_velocity_covariance[0] = 0.02; // Set the covariance of the angular velocity
 imu_msg.linear_acceleration_covariance[0] = 0.04; // Set the covariance of the linear acceleration*/


 nh.subscribe(sub_b);
 nh.subscribe(sub_one_int);
}


void loop(){
  wheel_nav_euler();
  mechanism();
  
}


void mechanism(){
 Serial.print(clr_ball);
 Serial.print("   ");
 Serial.print(ros_pb_var);
 Serial.print("   ");
 digitalWrite(BDDIR, LOW);
 /*Serial.print("ros: ");
 Serial.print(ros_pb_var);
 Serial.print(" clr: ");
 Serial.println(clr_ball);*/
  if (ros_pb_var == 10 && r1 == 1) {
   r1 = 0;
   if(clr_ball==1||clr_ball==2||clr_ball==3){
     esc1.writeMicroseconds(1650);
     esc2.writeMicroseconds(1350);
     //Serial.println("Into loop");
     delay(700);
     analogWrite(BDPWM, 255);
     delay(580);
     analogWrite(BDPWM, 0);
     esc1.writeMicroseconds(1500);
     esc2.writeMicroseconds(1500);
     r2 = 1;
     if (clr_ball == 3) {
       delay(1000);
       analogWrite(BDPWM, 255);
       delay(780);
       analogWrite(BDPWM, 0);
       r1 = 1;
       clr_ball = 4;
     }
     ros_pb_var = 0;
   }
 }
 if (ros_pb_var == 20 && r2 == 1) {
   analogWrite(BDPWM, 255);
   delay(780);
   analogWrite(BDPWM, 0);
   r1 = 1;
   r2 = 0;
   ros_pb_var = 0;
   clr_ball = 0;
 }
}
void wheel_nav_euler()
{
    Kinematics::output rpm;


   // if (Serial.available()>0) {//getting velocities as the input from Serial Monitor
   //   String input = Serial.readStringUntil('\n');
   //   int space1 = input.indexOf(' ');
   //   int space2 = input.indexOf(' ', space1 + 1);
   
   //   String menu1_str = input.substring(0, space1);
   //   String menu2_str = input.substring(space1 + 1, space2);
   //   String menu3_str = input.substring(space2 + 1);
   
   //   linear_vel_x = menu1_str.toFloat();
   //   linear_vel_y = menu2_str.toFloat();
   //   angular_vel_z = menu3_str.toFloat();
   // }


   rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);


   Serial.print("FRONT LEFT MOTOR: ");
   Serial.print(rpm.motor1);
   Serial.print(" FRONT RIGHT MOTOR: ");
   Serial.print(rpm.motor2);
   Serial.print(" REAR LEFT MOTOR: ");
   Serial.print(rpm.motor3);
   Serial.print(" REAR RIGHT MOTOR: ");
   Serial.print(rpm.motor4);


   rpm1 = rpm.motor1;
   rpm2 = rpm.motor2;
   rpm3 = rpm.motor3;
   rpm4 = rpm.motor4;


   int pwm1 = kinematics.rpmToPWM(rpm.motor1);
   int pwm2 = kinematics.rpmToPWM(rpm.motor2);
   int pwm3 = kinematics.rpmToPWM(rpm.motor3);
   int pwm4 = kinematics.rpmToPWM(rpm.motor4);


   Serial.print("PWM: ");
   Serial.print(pwm1);
   Serial.print(" ");
   Serial.print(pwm2);
   Serial.print(" ");
   Serial.print(pwm3);
   Serial.print(" ");
   Serial.println(pwm4);
   //Serial.println(pulse1);


   (rpm.motor1 > 0)?digitalWrite(DIR1,HIGH):digitalWrite(DIR1,LOW);
   (rpm.motor2 > 0)?digitalWrite(DIR2,HIGH):digitalWrite(DIR2,LOW);
   (rpm.motor3 > 0)?digitalWrite(DIR3,HIGH):digitalWrite(DIR3,LOW);
   (rpm.motor4 > 0)?digitalWrite(DIR4,HIGH):digitalWrite(DIR4,LOW);


   analogWrite(PWM1,abs(pwm1));
   analogWrite(PWM2,abs(pwm2));
   analogWrite(PWM3,abs(pwm3));
   analogWrite(PWM4,abs(pwm4));
 
    pulse1_msg.data = pulse1;
    pulse2_msg.data = pulse2;
    pulse3_msg.data = pulse3;
    pulse4_msg.data = pulse4;

    frontleft_pub.publish(&pulse1_msg);
    frontright_pub.publish(&pulse2_msg);
    rearleft_pub.publish(&pulse3_msg);
    rearright_pub.publish(&pulse4_msg);

// lcd.setCursor(0,0); //printing the encoder values in LCD
// lcd.print("0");
// lcd.setCursor(9,0);
// lcd.print("0");
// lcd.setCursor(0,1);
// lcd.print("0");
// lcd.setCursor(9,1);
// lcd.print("0");

   
    if (updateSensorData)  //Keep the updating of data as a separate task
    {
      mySensor.updateEuler();        //Update the Euler data into the structure of the object
      mySensor.updateCalibStatus();  //Update the Calibration Status
      mySensor.updateAccel();        //Update the Accelerometer data
      mySensor.updateLinearAccel();  //Update the Linear Acceleration data
      mySensor.updateCalibStatus();  //Update the Calibration Status
      mySensor.updateGyro();
      mySensor.updateQuat();
      updateSensorData = false;
    }
    
    if ((millis() - lastStreamTime) >= streamPeriod)
    {
      lastStreamTime = millis();
 
      imu_msg.orientation.x = Pi*mySensor.readEulerPitch()/180;
      imu_msg.orientation.y = Pi*mySensor.readEulerRoll()/180;
      imu_msg.orientation.z = Pi*mySensor.readEulerHeading()/180;
      imu_msg.orientation.w = euler_w;
 
      imu_msg.angular_velocity.x = Pi*mySensor.readGyro(X_AXIS);
      imu_msg.angular_velocity.y = Pi*mySensor.readGyro(Y_AXIS);
      imu_msg.angular_velocity.z = Pi*mySensor.readGyro(Z_AXIS);
 
      imu_msg.linear_acceleration.x = mySensor.readLinearAcceleration(X_AXIS);
      imu_msg.linear_acceleration.y = mySensor.readLinearAcceleration(Y_AXIS);
      imu_msg.linear_acceleration.z = mySensor.readLinearAcceleration(Z_AXIS);
 
      imu_msg.header.stamp = nh.now();
      imu_pub.publish(&imu_msg);
 
      nh.spinOnce();
 
      updateSensorData = true;
    }
   //nh.spinOnce();
}


void readEncoder1() {
 (digitalRead(ENCB1)>0)? pulse1++ : pulse1-- ;
}


void readEncoder2() {
 (digitalRead(ENCB2)>0)? pulse2++ : pulse2-- ;
}


void readEncoder3() {
 (digitalRead(ENCB3)>0)? pulse3++ : pulse3-- ;
}


void readEncoder4() {
 (digitalRead(ENCB4)>0)? pulse4++ : pulse4-- ;
}


void Euler() //This code is looped forever
{
 if ((millis() - lastStreamTime) >= streamPeriod)
 {
   lastStreamTime = millis();
   mySensor.updateEuler();        //Update the Euler data into the structure of the object
   mySensor.updateCalibStatus();  //Update the Calibration Status
   euler_z=Pi*mySensor.readEulerHeading()/180;
   euler_y=Pi*mySensor.readEulerRoll()/180;
   euler_x=Pi*mySensor.readEulerPitch()/180;


   Serial.print(" H: ");
   Serial.print(euler_z); //Heading data
   Serial.print("rad ");


   Serial.print(" R: ");
   Serial.print(euler_y); //Roll data
   Serial.print("rad");


   Serial.print(" P: ");
   Serial.print(euler_x); //Pitch data
   Serial.print("rad ");


   Serial.print(" G: ");
   Serial.print(mySensor.readGyroCalibStatus());   //Gyroscope Calibration Status (0 - 3)


   Serial.print(" S: ");
   Serial.print(mySensor.readSystemCalibStatus());   //System Calibration Status (0 - 3)


   Serial.println();
 }
}


// void readBDEncoder1() {
//  (digitalRead(bdencB[0]) > 0) ? bdencoderCount[0]++ : bdencoderCount[0]--;
// }


// void readBDEncoder2() {
//  (digitalRead(bdencB[1]) > 0) ? bdencoderCount[1]++ : bdencoderCount[1]--;
// }
