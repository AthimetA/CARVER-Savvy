#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <control_LIB.h>
#include <ShiftRegister74HC595.h>

//serial 
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// micro-ROS library
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS2 message types
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

// General parameters
#include <General_params.h>

// Adafruit
#include <Adafruit_Sensor.h> 
//------------------------------------function declaration------------------------------------
void without_uROS();
//function softcheck
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){without_uROS();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//define pin numbers
#define pinStatusMotor GPIO_NUM_36
#define pinEmerSwitch GPIO_NUM_39
#define pinBypassSwitch GPIO_NUM_15
#define pinTriggerSwitch GPIO_NUM_34
#define pinSelectorSwitch_1 GPIO_NUM_35
#define pinSelectorSwitch_2 GPIO_NUM_16
#define pinButtonSwitch GPIO_NUM_0
#define pinVoltage GPIO_NUM_13
#define pinBumperSwitch GPIO_NUM_12
int latchPin = 2;
int clockPin = 5;
int dataPin = 18;
ShiftRegister74HC595<1> sr(dataPin, clockPin, latchPin);

//define encoder 
int pinEncA2 = 14; 
int pinEncB2 = 19;  

int pinEncA1 = 17;    // Ture B
int pinEncB1 = 4;   // Ture A

//odometry
// #define NORMALIZE(_z) atan2(sin(_z), cos(_z))
// char base_link[] = "/base_footprint";
// char odom[] = "/odom";
// int intervalTf = 100;
// double preIntervelTfMillis = 0;
// double x = 0.0;
// double y = 0.0;
// double theta = 0.0;
// double angular = 0.0;
// bool isDetectError = false;

// Read voltage Battery
float V_Batt_Full = 26.52; // Not change
float V_Read_Max = 2.457;  // Not change
float Gain_ana = 1.099; // 1.099
float sampling_time = 200; //hz

// PID parameters
uint16_t Offset_Read_Analog = 0; // ถ้า V ไม่ตรงปรับ offset ตรงนี้ แปรผันตรง
// float r_wheel = 0.125;
float d = 0.6;
float res_encoder = 2048;
float qei = 2;
float gear = 300;
float pi = 3.14159265359;
String motor_status = "on";
String controller_state = "Init";
float kp[1];
float ki[1];
float kd[1];
int st[1];
float to[1];
long current_loop = 0;
float timeout = 1000;

/* Set the delay between fresh samples */
// ----------------- Global variables -----------------
// ----------------- ROS2 -----------------
//ENCL
rcl_publisher_t ENCL_publisher;
std_msgs__msg__Int16 ENCL_msg;

//ENCR
rcl_publisher_t ENCR_publisher;
std_msgs__msg__Int16 ENCR_msg;

//wheelL vel
rcl_publisher_t wheelL_vel_publisher;
std_msgs__msg__Float64 wheelL_vel_msg;

//wheelR vel
rcl_publisher_t wheelR_vel_publisher;
std_msgs__msg__Float64 wheelR_vel_msg;

//imu
rcl_publisher_t IMU_publisher;
sensor_msgs__msg__Imu IMU_msg;

//KP
rcl_publisher_t KP_publisher;
std_msgs__msg__Float64 KP_msg;

//KI
rcl_publisher_t KI_publisher;
std_msgs__msg__Float64  KI_msg;

//KD
rcl_publisher_t KD_publisher;
std_msgs__msg__Float64  KD_msg;

//cmd_vel
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist Speed_msg;


rclc_executor_t executor_pub; // Executor for publisher

rclc_executor_t executor_sub; // Executor for subscriber

rclc_support_t support; // Support for micro-ROS

rcl_allocator_t allocator; // Allocator for micro-ROS

rcl_node_t node; // Node for micro-ROS

rcl_timer_t timer;

int16_t Sub_speedL = 0;
int16_t Sub_speedR = 0;

// ----------------- control -----------------
// encoder
ESP32Encoder encoderL;
ESP32Encoder encoderR;

float encoderLrad = 0;
float encoderRrad = 0;

unsigned long timestamp;
bool encoder2Paused = false;
// PID parameters
PIDparam pidParameter1;
PIDparam pidParameter2;

// dcMotor
dcMotor motorL;
dcMotor motorR;

// velocity
velocity robotVelocityCmd;
velocity robotVelocityEnc;

PID pidController1(&pidParameter1.encInput, &pidParameter1.output, &pidParameter1.setPoint,
                   0, 0, 0, DIRECT);
PID pidController2(&pidParameter2.encInput, &pidParameter2.output, &pidParameter2.setPoint,
                   0, 0, 0, DIRECT);

int intervalPID = 10;
double preIntervelMillis = 0;
double prePubMillis = 0;
double preDetectError = 0;
double preInput = 0;

double limitOffset1 = 4096;
double limitOffset2 = 4096;

double ikOut[3];

// ------------------------------------ros2 configuration------------------------------------

void subscription_callback(const void *msgin) 
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED

	Sub_speedL = (msg->linear.x/ROBOT_WHEEL_RADIUS) - (msg->angular.z * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
  	Sub_speedR = (msg->linear.x/ROBOT_WHEEL_RADIUS) + (msg->angular.z * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
	robotVelocityCmd.Vx = msg->linear.x;
	robotVelocityCmd.Vy = msg->linear.y;
	robotVelocityCmd.w = msg->angular.z;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
	ENCR_msg.data = control.getCountEnc(&encoderR);
	ENCL_msg.data = control.getCountEnc(&encoderL);

	wheelL_vel_msg.data = encoderLrad;
	wheelR_vel_msg.data = encoderRrad;

	// IMU_msg.angular_velocity.x = encoderLrad;

	RCSOFTCHECK(rcl_publish(&ENCL_publisher, &ENCL_msg, NULL));
	RCSOFTCHECK(rcl_publish(&ENCR_publisher, &ENCR_msg, NULL));
	RCSOFTCHECK(rcl_publish(&wheelL_vel_publisher, &wheelL_vel_msg, NULL));
	RCSOFTCHECK(rcl_publish(&wheelR_vel_publisher, &wheelR_vel_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&IMU_publisher, &IMU_msg, NULL));
	RCSOFTCHECK(rcl_publish(&KP_publisher, &KP_msg, NULL));
	RCSOFTCHECK(rcl_publish(&KI_publisher, &KI_msg, NULL));
	RCSOFTCHECK(rcl_publish(&KD_publisher, &KD_msg, NULL));
  }

}

void uROSsetup()	
{
	set_microros_serial_transports(Serial);
	allocator = rcl_get_default_allocator();
	
	//create init_options
	rclc_support_init(&support, 0, NULL, &allocator);
	
	//create node
	 RCCHECK(rclc_node_init_default(&node, "mini_project_PMZB_node", "", &support));
	
	//create publisher
	RCCHECK(rclc_publisher_init_default(&ENCL_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rawENCL"));
	
	RCCHECK(rclc_publisher_init_default(&ENCR_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rawENCR"));

	RCCHECK(rclc_publisher_init_default(&wheelL_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "wheelL_vel"));

	RCCHECK(rclc_publisher_init_default(&wheelR_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "wheelR_vel"));

	// RCCHECK(rclc_publisher_init_best_effort(&IMU_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "IMU"));

	RCCHECK(rclc_publisher_init_default(&KP_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "KP"));

	RCCHECK(rclc_publisher_init_default(&KI_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "KI"));

	RCCHECK(rclc_publisher_init_default(&KD_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "KD"));

	//create subscriber
	RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

	//create timer
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	//create executor
	RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
	
	RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &Speed_msg, &subscription_callback, ON_NEW_DATA));
	ENCR_msg.data = 0;
	ENCL_msg.data = 0;

	wheelL_vel_msg.data = 0;
	wheelR_vel_msg.data = 0;
}

void without_uROS() 
{
	  while (1) {
	Serial.println("ROS2 is not working");
	delay(1000);
  }

}

void uROSloop()
{
	RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10)));
	RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10)));
}	

void taskOne( void * parameter )
{
    while(true)
    {
        if (millis() - preInput >= 10)
        {
            preInput = millis();
        }
        
    }
}

//------------------------------------control configuration------------------------------------

void start_motor()
{
  sr.set(1, HIGH);
  delay(1000);
  sr.set(1, LOW);
}

void stop_motor()
{
  sr.set(0, HIGH);
  delay(1000);
  sr.set(0, LOW);
}

void interlock()
{
  sr.set(4, HIGH);
  sr.set(5, HIGH);
  delay(1000);
  sr.set(4, LOW);
  sr.set(5, LOW);
  
}
//move function calculate.h
float rad_to_enc(float w)
{
//   return (w * res_encoder * qei ) / (sampling_time * 2 * pi);
return (w * res_encoder * qei ) / (2 * pi);
}

float enc_to_rad(float enc)
{
  return ( (enc * sampling_time) / ( res_encoder * qei ) ) * 2 * pi;
//   return ( enc / ( res_encoder * qei ) ) * 2 * pi;
}
//move function calculate.h
void controlSetup()
{
	//move parameter to general_params.h
	//parameter setup
	pidParameter1.Kp = 0.5;//3;  //1;   //4
    pidParameter1.Ki = 0;//0.5; //8;
    pidParameter1.Kd = 0.0;
    pidParameter1.sampleTime = intervalPID;

    pidParameter2.Kp = 0.5;//3;  //1;   //4
    pidParameter2.Ki = 0;//0.5; //8;
    pidParameter2.Kd = 0.0;
    pidParameter2.sampleTime = intervalPID;

    motorR.pwmChannel = 1;
    motorR.pwmPin = 26;
    motorR.outAPin = 33;
    motorR.outBPin = 32;

    motorL.pwmChannel = 0;
    motorL.pwmPin = 25;
    motorL.outAPin = 23;
    motorL.outBPin = 27;
	//encoder setup
	control.initialEnc(&encoderL, "enc1", pinEncA1, pinEncB1, 30); // encoder pin 17, 4
    control.initialEnc(&encoderR, "enc2", pinEncB2, pinEncA2, 30); // encoder pin 14, 19
	//motor setup
	control.initialMotor(&motorL, 1000, 10);
    control.initialMotor(&motorR, 1000, 10);
	//PID setup
	control.initialPID(&pidController1, &pidParameter1, limitOffset1);
    control.initialPID(&pidController2, &pidParameter2, limitOffset2);
	control.zeroOutputSum(&pidController1);
	control.zeroOutputSum(&pidController2);
}

void controlLoop()
{
	// PID 100 HZ
	if (millis() - preIntervelMillis >= intervalPID) 
	{
		preIntervelMillis = millis();
		// get encoder count
		encoderLrad = enc_to_rad(control.getCountEnc(&encoderL));//rad/s
		encoderRrad = enc_to_rad(control.getCountEnc(&encoderR));//rad/s
		// get velocity command
		// robotVelocityCmd.v1 = Sub_speedL;
		// robotVelocityCmd.v2 = Sub_speedR;
		robotVelocityCmd.v1 = 0.5;
		robotVelocityCmd.v2 = 0.5;
		// inverse kinematics
		pidParameter1.setPoint = rad_to_enc(robotVelocityCmd.v1);// 4096 pulse per revolution
		pidParameter2.setPoint = rad_to_enc(robotVelocityCmd.v2);
		// setpoint
		control.setpoint(&pidController1, &pidParameter1, &encoderL);
		control.setpoint(&pidController2, &pidParameter2, &encoderR);
		//stop
		Serial.print(String(pidParameter1.output));
		Serial.print(" ");
		Serial.println(String(pidParameter2.output));
	// if (robotVelocityCmd.Vx == 0 && robotVelocityCmd.w == 0)
    // {
    //     control.drive(&motorL, 0);
    //     control.drive(&motorR, 0);
	// 	control.zeroOutputSum(&pidController1);
	// 	control.zeroOutputSum(&pidController2);
    // }
	// else
    //     {           
			// drive
			control.drive(&motorL, pidParameter1.output);
			control.drive(&motorR, pidParameter2.output);
		// }
	}
	
}


//------------------------------------------------- Main -------------------------------------------------

void setup(){
	
	Serial.begin(115200);
	// encoder setup
	controlSetup();

	Serial.println("Encoder Start = " + String((int32_t)control.getCountEnc(&encoderL)));
	// dualcore
	xTaskCreate(&taskOne, "myTask1",  10000, NULL, 1, NULL);
	// micro-ROS setup
	uROSsetup();

}

void loop(){

	// micro-ROS loop
	uROSloop();
	// control loop
	// controlLoop();
	// check every 0.5 seconds test motor
	// if (millis() - timestamp >= 10) {
	// 	// Serial.println("Encoder count = " + String((int32_t)control.getCountEnc(&encoderL)) + " " + String((int32_t)control.getCountEnc(&encoderR)));
	// 	timestamp = millis();
	// 	// encoderLrad = enc_to_rad(control.getCountEnc(&encoderL));
	// 	// encoderRrad = enc_to_rad(control.getCountEnc(&encoderR));
	// 	encoderLrad = control.getIntervalEnc(&encoderL);
	// 	encoderRrad = control.getIntervalEnc(&encoderR);
	// 	Serial.println("Encoder rad = " + String(encoderLrad) + " " + String(encoderRrad));
	// }
}