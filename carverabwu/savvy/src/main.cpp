#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <control_LIB.h>
#include <ShiftRegister74HC595.h>
///test
#include <ESP32Encoder.h>
ESP32Encoder encoder;
ESP32Encoder encoder2;
///test
// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
// #include <calculate.h>
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
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
#include <std_msgs/msg/float32.h>
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
TaskHandle_t Task1;
TaskHandle_t Task2;
//define encoder 
int pinEncA1 = 14;    // Ture B
int pinEncB1 = 19;   // Ture A

int pinEncA2 = 4; 
int pinEncB2 = 17;  



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
float sampling_time = 25; //hz

// PID parameters
uint16_t Offset_Read_Analog = 0; // ถ้า V ไม่ตรงปรับ offset ตรงนี้ แปรผันตรง
// float r_wheel = 0.125;
float d = 0.6;
float res_encoder = 2048.0;
float qei = 2.0;
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
std_msgs__msg__Float32 wheelL_vel_msg;

//wheelR vel
rcl_publisher_t wheelR_vel_publisher;
std_msgs__msg__Float32 wheelR_vel_msg;

//imu
rcl_publisher_t IMU_publisher;
sensor_msgs__msg__Imu IMU_msg;
//imu_data
rcl_publisher_t IMU_yaw_publisher;
std_msgs__msg__Float32 IMU_yaw_msg;

rcl_publisher_t IMU_vz_publisher;
std_msgs__msg__Float32 IMU_vz_msg;

rcl_publisher_t IMU_ax_publisher;
std_msgs__msg__Float32 IMU_ax_msg;

//input controlL
rcl_publisher_t inputL_publisher;
std_msgs__msg__Float32 inputcontrolL_msg;

//input controlR
rcl_publisher_t inputR_publisher;
std_msgs__msg__Float32 inputcontrolR_msg;

rcl_publisher_t outputL_publisher;
std_msgs__msg__Int16 outputcontrolL_msg;

//output controlR
rcl_publisher_t outputR_publisher;
std_msgs__msg__Int16 outputcontrolR_msg;

//cmd_vel
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist Speed_msg;


rclc_executor_t executor_pub; // Executor for publisher

rclc_executor_t executor_sub; // Executor for subscriber

rclc_support_t support; // Support for micro-ROS

rcl_allocator_t allocator; // Allocator for micro-ROS

rcl_node_t node; // Node for micro-ROS

rcl_timer_t timer;

float Sub_speedL = 0;
float Sub_speedR = 0;

// ----------------- control -----------------
// encoder
ESP32Encoder encoderL;
ESP32Encoder encoderR;


int32_t rawencL = 0;
int32_t rawencR = 0;

float encoderLrad = 0;
float encoderRrad = 0;
float feedfowardL = 0;
float feedfowardR = 0;

float IMU_data[10];

unsigned long timestamp;
bool encoder2Paused = false;
// PID parameters
PIDparam pidParameter1;
PIDparam pidParameter2;
PIDparam pidParameter3;
PIDparam pidParameter4;
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
PID pidController3(&pidParameter3.encInput, &pidParameter3.output, &pidParameter3.setPoint,
				   0, 0, 0, DIRECT);	
PID pidController4(&pidParameter4.encInput, &pidParameter4.output, &pidParameter4.setPoint,
				   0, 0, 0, DIRECT);

double preIntervelMillis = 0;
double prePubMillis = 0;
double preDetectError = 0;
double preInput = 0;
int countmicro = 0;
int counttimer = 0;
double limitOffset1 = 0;
double limitOffset2 = 0;

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
	// ENCR_msg.data = counttimer; 
	// ENCL_msg.data = counttimer;
	// outputcontrolL_msg.data = pidParameter1.output;
	// outputcontrolR_msg.data = pidParameter2.output;
	// outputcontrolL_msg.data = feedfowardL;
	// outputcontrolR_msg.data = feedfowardR;
	
	// inputcontrolL_msg.data = robotVelocityCmd.v1;
	// inputcontrolR_msg.data = robotVelocityCmd.v2;
	inputcontrolL_msg.data = Sub_speedL;
	inputcontrolR_msg.data = Sub_speedR;
	wheelL_vel_msg.data = encoderLrad;
	wheelR_vel_msg.data = encoderRrad;
	// IMU_msg.angular_velocity.x = -1.0*IMU_data[0];
	// IMU_msg.angular_velocity.y = IMU_data[2];
	IMU_vz_msg.data = IMU_data[1];
	IMU_ax_msg.data = -1.0* IMU_data[3];
	// IMU_msg.linear_acceleration.y = IMU_data[5];
	// IMU_msg.linear_acceleration.z = IMU_data[4];
	// IMU_msg.orientation.w = IMU_data[6];
	// IMU_msg.orientation.x = IMU_data[7];
	// IMU_msg.orientation.y = IMU_data[8];
	IMU_yaw_msg.data = IMU_data[8];
	// RCSOFTCHECK(rcl_publish(&ENCL_publisher, &ENCL_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&ENCR_publisher, &ENCR_msg, NULL));
	RCSOFTCHECK(rcl_publish(&wheelL_vel_publisher, &wheelL_vel_msg, NULL));
	RCSOFTCHECK(rcl_publish(&wheelR_vel_publisher, &wheelR_vel_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&IMU_publisher, &IMU_msg, NULL));
	RCSOFTCHECK(rcl_publish(&IMU_yaw_publisher, &IMU_yaw_msg, NULL));
	RCSOFTCHECK(rcl_publish(&IMU_vz_publisher, &IMU_vz_msg, NULL));
	RCSOFTCHECK(rcl_publish(&IMU_ax_publisher, &IMU_ax_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&IMU_publisher, &IMU_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&outputL_publisher, &outputcontrolL_msg, NULL));
	// RCSOFTCHECK(rcl_publish(&outputR_publisher, &outputcontrolR_msg, NULL));
	RCSOFTCHECK(rcl_publish(&inputL_publisher, &inputcontrolL_msg, NULL));
	RCSOFTCHECK(rcl_publish(&inputR_publisher, &inputcontrolR_msg, NULL));
	// counttimer++;
  }
}

void uROSsetup()	
{
	set_microros_serial_transports(Serial);
	// IPAddress ip(192, 168, 12, 1);
    // set_microros_wifi_transports("TrashX", "00000000", ip, 8888);
	allocator = rcl_get_default_allocator();
	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	//create node
	RCCHECK(rclc_node_init_default(&node, "mini_project_PMZB_node", "", &support));
	  // sync time
  	// rmw_uros_sync_session(1000);
	//create publisher
	// RCCHECK(rclc_publisher_init_default(&ENCL_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rawENCL"));
	// RCCHECK(rclc_publisher_init_default(&ENCR_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rawENCR"));
	RCCHECK(rclc_publisher_init_default(&wheelL_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheelL_vel"));
	RCCHECK(rclc_publisher_init_default(&wheelR_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheelR_vel"));
	// RCCHECK(rclc_publisher_init_default(&outputL_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "outputL"));
	// RCCHECK(rclc_publisher_init_default(&outputR_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "outputR"));
	// RCCHECK(rclc_publisher_init_default(&inputL_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "inputL"));
	// RCCHECK(rclc_publisher_init_default(&inputR_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "inputR"));
	RCCHECK(rclc_publisher_init_default(&IMU_yaw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_yaw"));
	RCCHECK(rclc_publisher_init_default(&IMU_vz_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_vz"));
	RCCHECK(rclc_publisher_init_default(&IMU_ax_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_ax"));
//   RCCHECK(rclc_publisher_init_default(&IMU_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "IMU"));
	//create subscriber
	RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "carversavvy_cmd_vel"));
	//create timer
	const unsigned int timer_timeout = 1000/sampling_time; //25 HZ
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
	//create executor
	RCCHECK(rclc_executor_init(&executor_pub, &support.context, 6, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
	// RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor_pub, &subscriber, &Speed_msg, &subscription_callback, ON_NEW_DATA));
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
	RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(1)));
	delay(1);
	// RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)));

}	

//----------------------------------------IMU--------------------------------------------------
// Here is where you define the sensor outputs you want to receive

void setReports(void) {
//   Serial.println("Setting desired reports");
//   if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
//     Serial.println("Could not enable accelerometer");
//   }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    // Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    // Serial.println("Could not enable linear acceleration");
  }
  if(!bno08x.enableReport(SH2_ROTATION_VECTOR)){
	// Serial.println("Could not enable rotation vector");
  }
}

//---------------------feedforward control------------------------------------
float InverseTFofMotorL(float Velo, float PredictVelo)
{
	static float VeloLast = 0.0;
	static float Voltage = 0.0;
	static float VoltageLast = 0.0;
	static float Pwm = 0;
	// Voltage = (PredictVelo - (1.298649403776808*Velo) + (0.413830007244888*VeloLast) - (0.492093238713741*VoltageLast))/0.660367603263632;
	if (PredictVelo > 0) 
	{
		Voltage = ((PredictVelo*2.2937 +3.5326)) - ((2.2937 *Velo));
	}
	else
	{
		Voltage = ((2.076*PredictVelo -3.1637) )- ((2.076*Velo) );
	}
	// Voltage = PredictVelo - ((2.4677*Velo) +  1.8768/2.0);
	Pwm = (Voltage * 255)/24.0;
	// VoltageLast = Voltage;
	// VeloLast = Velo;
	return Pwm;
}

float InverseTFofMotorR(float Velo, float PredictVelo)
{
	static float VeloLast = 0.0;
	static float Voltage = 0.0;
	static float VoltageLast = 0.0;
	static float Pwm = 0;
	// Voltage = (PredictVelo - (1.298649403776808*Velo) + (0.413830007244888*VeloLast) - (0.492093238713741*VoltageLast))/0.660367603263632;
	if (PredictVelo > 0)
	{
		Voltage = ((PredictVelo*2.2501 +2.7908)) - ((2.2501 *Velo));
	}
	else
	{
		Voltage = ((2.1973*PredictVelo -2.9272) )- ((2.1973*Velo) );
	}
	Pwm = (Voltage * 255)/24.0;
	// VoltageLast = Voltage;
	// VeloLast = Velo;
	return Pwm;
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
return (w * res_encoder * qei ) / (2 * M_PI);
}

float enc_to_rad(float enc)
{
  return ( (enc * sampling_time) / ( res_encoder * qei ) ) * 2.0 * M_PI;
  // return ( enc / ( res_encoder * qei ) ) * 2 * pi;
}
//move function calculate.h
void controlSetup()
{
	//IMU setup--------------------------------------------------------------------------------------------------------------------
	while (!Serial)
    	delay(10);// will pause Zero, Leonardo, etc until serial console opens
	// Serial.println("Adafruit BNO08x test!");
	// Try to initialize!
	if (!bno08x.begin_I2C()) {
    // Serial.println("Failed to find BNO08x chip");
    	while (1) {
      	delay(10);
    	}
 	}
	// Serial.println("BNO08x Found!");
	for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    // Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    // Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    // Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    // Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    // Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  	}
  	setReports();
  	// Serial.println("Reading events");
  	delay(100);
	//IMU setup--------------------------------------------------------------------------------------------------------------------	
	//move parameter to general_params.h
	//parameter setup
  pidParameter1.Kp = 0.034;//3;  //1;   //4 
  pidParameter1.Ki = 0.001;//0.18;//0.5; //8;
  pidParameter1.Kd = 0.1;
  pidParameter1.sampleTime = 1000/sampling_time;

  pidParameter2.Kp = 0.034;//* 0.45;//3;  //1;   //4 0.03
  pidParameter2.Ki = 0.001;//1.2*0.06/2.8;//0.5; //8;0.18
  pidParameter2.Kd = 0.1;
  pidParameter2.sampleTime = 1000/sampling_time;

  pidParameter3.Kp = 0.048;//3;  //1;   //4
  pidParameter3.Ki = 0.002;//0.18;//0.5; //8;
  pidParameter3.Kd = 0.0;
  pidParameter3.sampleTime = 1000/sampling_time;

  pidParameter4.Kp = 0.048;//* 0.45;//3;  //1;   //4 0.03
  pidParameter4.Ki = 0.002;//1.2*0.06/2.8;//0.5; //8;0.18
  pidParameter4.Kd = 0.0;
  pidParameter4.sampleTime = 1000/sampling_time;

  motorR.pwmChannel = 1;
  motorR.pwmPin = 25;
  motorR.outAPin = 27;
  motorR.outBPin = 23;

  motorL.pwmChannel = 0;
  motorL.pwmPin = 26;
  motorL.outAPin = 33;
  motorL.outBPin = 32;
	//encoder setup--------------------------------------------------------------------------------------------------------------------
	control.initialEnc(&encoderL, "enc1", pinEncA1, pinEncB1, 4096); // encoder pin 17, 4
  	control.initialEnc(&encoderR, "enc2", pinEncA2, pinEncB2, 4096); // encoder pin 14, 19
	control.initialEnc(&encoder, "enc1", pinEncA1, pinEncB1, 4096);
	control.initialEnc(&encoder2, "enc2", pinEncA2, pinEncB2, 4096);
	//encoder setup--------------------------------------------------------------------------------------------------------------------
	// //motor setup
	control.initialMotor(&motorL, 100, 8);
  	control.initialMotor(&motorR, 100, 8);
	// //PID setup
	control.initialPID(&pidController1, &pidParameter1, limitOffset1);
 	control.initialPID(&pidController2, &pidParameter2, limitOffset2);
	control.initialPID(&pidController3, &pidParameter3, limitOffset1);	
	control.initialPID(&pidController4, &pidParameter4, limitOffset2);
	control.zeroOutputSum(&pidController1);
	control.zeroOutputSum(&pidController2);
	control.zeroOutputSum(&pidController3);
	control.zeroOutputSum(&pidController4);
}

void controlLoop()
{
	// PID 100 HZ
	if (millis() - preIntervelMillis >= (1000/sampling_time)) 
	{
		preIntervelMillis = millis();
		// Serial.println("This Task run on Core: " + String(xPortGetCoreID()));
		// get IMU data

		if (bno08x.wasReset()) {
    		// Serial.print("sensor was reset ");
    		// setReports();
  			}		
  		if (!bno08x.getSensorEvent(&sensorValue)) {
    		return;
  			}
  		switch (sensorValue.sensorId) {
			case SH2_GYROSCOPE_CALIBRATED:
				// Serial.print("Gyro - x: "+ str(sensorValue.un.gyroscope.x));
				IMU_data[0] = sensorValue.un.gyroscope.x;
				// Serial.print(" y: "+ str(sensorValue.un.gyroscope.y));
				IMU_data[1] = sensorValue.un.gyroscope.y;
				// Serial.print(" z: "+ str(sensorValue.un.gyroscope.z));
				IMU_data[2] = sensorValue.un.gyroscope.z;
				break;
			case SH2_LINEAR_ACCELERATION:
				// Serial.print("Linear Acceration - x: "+ str(-1.0 * sensorValue.un.linearAcceleration.x));
				IMU_data[3] = -1.0 * sensorValue.un.linearAcceleration.x;
				// Serial.print(" y: " + str(-1.0 * sensorValue.un.linearAcceleration.y);
				IMU_data[4] = -1.0 * sensorValue.un.linearAcceleration.y;
				// Serial.print(" z: "+ str(-1.0 * sensorValue.un.linearAcceleration.z);
				IMU_data[5] = -1.0 * sensorValue.un.linearAcceleration.z;
				break;
			case SH2_ROTATION_VECTOR:
				// Serial.print("Rotation Vector - r: "+ str(sensorValue.un.rotationVector.real));
				IMU_data[6] = sensorValue.un.rotationVector.real;
				// Serial.print(" i: "+ str(sensorValue.un.rotationVector.i));
				IMU_data[7] = sensorValue.un.rotationVector.i;
				// Serial.print(" j: "+ str(sensorValue.un.rotationVector.j));
				IMU_data[8] = sensorValue.un.rotationVector.j;
				// Serial.print(" k: "+ str(sensorValue.un.rotationVector.k));
				IMU_data[9] = sensorValue.un.rotationVector.k;
				break;
  		}
		// get velocity command
		robotVelocityCmd.v1 = Sub_speedL;
		robotVelocityCmd.v2 = Sub_speedR;
		// robotVelocityCmd.v1 = 2 * M_PI;
		// robotVelocityCmd.v2 = 2 * M_PI;
		// // inverse kinematics
		pidParameter1.setPoint = rad_to_enc(robotVelocityCmd.v1);// 4096 pulse per revolution
		pidParameter2.setPoint = rad_to_enc(robotVelocityCmd.v2);
		pidParameter3.setPoint = rad_to_enc(robotVelocityCmd.v1);
		pidParameter4.setPoint = rad_to_enc(robotVelocityCmd.v2);
    // pidParameter1.setPoint = robotVelocityCmd.v1;
    // pidParameter2.setPoint = robotVelocityCmd.v2;
		encoderLrad = enc_to_rad(control.getIntervalEnc(&encoder)); 
    	encoderRrad = enc_to_rad(control.getIntervalEnc(&encoder2));
		feedfowardL = InverseTFofMotorL(encoderLrad, robotVelocityCmd.v1);
		feedfowardR = InverseTFofMotorR(encoderRrad, robotVelocityCmd.v2);
		// setpoint
		if (robotVelocityCmd.w != 0)
		{
			control.setpoint(&pidController3, &pidParameter3, &encoderL);
			control.setpoint(&pidController4, &pidParameter4, &encoderR);
			feedfowardL = feedfowardL +pidParameter3.output;
			feedfowardR = feedfowardR +pidParameter4.output;
		}
		else
		{
			control.setpoint(&pidController1, &pidParameter1, &encoderL);
			control.setpoint(&pidController2, &pidParameter2, &encoderR);
			feedfowardL = feedfowardL +pidParameter1.output;
			feedfowardR = feedfowardR +pidParameter2.output;
		}
		//feed forward control
		// feedfowardL = pidParameter1.output;
		// feedfowardR = pidParameter2.output;
		if (feedfowardL > 250)
		{
			feedfowardL = 250;
		}
		if (feedfowardR > 250)
		{
			feedfowardR = 250;
		}
		if (robotVelocityCmd.Vx == 0 && robotVelocityCmd.w == 0)
		{
			control.drive(&motorL, 0);
			control.drive(&motorR, 0);
			control.zeroOutputSum(&pidController1);
			control.zeroOutputSum(&pidController2);
			control.zeroOutputSum(&pidController3);
			control.zeroOutputSum(&pidController4);
		}
		else
		{           
		// // drive
			control.drive(&motorL, feedfowardL);
			control.drive(&motorR, feedfowardR);
		}
		// control.drive(&motorL, 250);
		// control.drive(&motorR, -250);

	// encoderLrad = control.getIntervalEnc(&encoderL); 
    // encoderRrad = control.getIntervalEnc(&encoderR);
    // rawencL = control.getCountEnc(&encoderL);  
    // rawencR = control.getCountEnc(&encoderR);
		}

	
}



//------------------------------------------------- Main -------------------------------------------------

void setup(){
	Serial.begin(115200);
	

	// dualcore
	pinMode(pinStatusMotor, INPUT);
  	pinMode(pinEmerSwitch, INPUT);
  	pinMode(pinBypassSwitch, INPUT);
  	pinMode(pinTriggerSwitch, INPUT);
  	pinMode(pinSelectorSwitch_1, INPUT);
  	pinMode(pinSelectorSwitch_2, INPUT);
  	pinMode(pinButtonSwitch, INPUT);
  	pinMode(pinVoltage, INPUT);
  	uROSsetup();
	controlSetup();
	start_motor();
	// stop_motor();
  	interlock();
	
}

void loop()
{
	controlLoop();
  uROSloop();
}
