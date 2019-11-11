///////////////////////////////////////////////////////////////////////////////////////////
/*
List of devices and ICs: 
	- Arduino MEGA 2560 Rev3
	- Pololu G2 High-Power Motor Driver 18v17 (https://www.pololu.com/product/2991)
	- Acutuonix T16-200-64-12-P (https://www.actuonix.com/T16-P-Mini-Track-Actuator-p/t16-p.htm)
*/
///////////////////////////////////////////////////////////////////////////////////////////

#include <ros.h> // Always include this header for serial communication with ROS
// #include <ArduinoHardware.h> // Optional for advanced options
#include <PID_v1.h> // https://playground.arduino.cc/Code/PIDLibrary/
#include <TimedAction.h> // https://playground.arduino.cc/Code/TimedAction/
#include <ros/time.h> // For generating timestamp that sync Arduino with roscore

// Built-in ROS Data Type
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
// #include <geometry_msgs/Twist.h>
#include <control_msgs/JointJog.h>
#include <std_msgs/Float32.h>

// Set up pins for potentiometer reading
#define M1_POT A0 // 10-bit (0-1023)
#define M2_POT A1 
#define M3_POT A2 
#define M4_POT A3 

// Set up pins for motor drivers
// PWM  
#define M1_PWM  2 
#define M2_PWM  3 
#define M3_PWM  4 
#define M4_PWM  5 
// Direction
#define M1_DIR 22 
#define M2_DIR 24 
#define M3_DIR 26 
#define M4_DIR 28 
// ON/OFF
#define M1_SLP 23 
#define M2_SLP 25 
#define M3_SLP 27 
#define M4_SLP 29 

// Interval time for different tasks in milliseconds 
// This link demonstrate simple codes for times action tasks similar to the library TimeAction.h 
// (https://www.norwegiancreations.com/2017/09/arduino-tutorial-using-millis-instead-of-delay/)
#define INTERVAL_LOW_LVL_CTRL 40 //25 Hz

// Switch between 0 or 1 to debug
#define DEBUG 0

///////////////////////////////////////////////////////////////////////////////////////////

// Global Constants 
float motor_max_speed = 64; // [mm/s]
float motor_stroke_length = 200; // [mm]

// Set up controlled variables
double motor_pos[4]; // [mm]

// Set up command variables
double des_motor_pos[4]; // [mm]
double motor_pwm[4]; // [-255, 255] duty cycle

// Setup output variables
float motor_lin_speed[4]; // [mm/s]

// Initialize array of all motor position (http://wiki.ros.org/rosserial/Overview/Messages)
sensor_msgs::JointState joint_state;
control_msgs::JointJog des_joint_state;
//creating the arrays for the message
char *name[] = {"motor_1", "motor_2", "motor_3", "motor_4"};
float vel[4];
float pos[4];
float eff[4];
char *des_name[] = {"motor_1", "motor_2", "motor_3", "motor_4"};
float des_pos[4];
float des_vel[4];

// initialize PID controller
double Kp = 10, Ki = 0.1, Kd=0;
PID motor1_PID(&motor_pos[0], &motor_pwm[0], &des_motor_pos[0], Kp, Ki, Kd, DIRECT);
PID motor2_PID(&motor_pos[1], &motor_pwm[1], &des_motor_pos[1], Kp, Ki, Kd, DIRECT);
PID motor3_PID(&motor_pos[2], &motor_pwm[2], &des_motor_pos[2], Kp, Ki, Kd, DIRECT);
PID motor4_PID(&motor_pos[3], &motor_pwm[3], &des_motor_pos[3], Kp, Ki, Kd, DIRECT);

///////////////////////////////////////////////////////////////////////////////////////////

// Set up Arduino as a node
ros::NodeHandle nh;

// Set up Publishers
ros::Publisher pubJointState("joint_state", &joint_state);
ros::Publisher pubDesJointState("des_joint_state", &des_joint_state);

// Callback Functions
void messageLEDCb( const std_msgs::Empty& toggle_msg) {
	digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // toggle the built-in LED
}

// void messagePosCmdCb( const control_msgs::JointJog& jog_msg) { 
// 	// See and example in https://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
// 	for (int i = 0; i < 4; i++) {
// 		des_joint_state.displacements[i] = jog_msg.displacements[i];
// 	}
// 	des_joint_state.displacements = jog_msg.displacements;
// 	pubDesJointState.publish( &des_joint_state );
// }

void messagePosCmdMotor1( const std_msgs::Float32& msg) {
	des_motor_pos[0] = msg.data;
}

void messagePosCmdMotor2( const std_msgs::Float32& msg) {
	des_motor_pos[1] = msg.data;
}

void messagePosCmdMotor3( const std_msgs::Float32& msg) {
	des_motor_pos[2] = msg.data;
}

void messagePosCmdMotor4( const std_msgs::Float32& msg) {
	des_motor_pos[3] = msg.data;
}

// Setup Subscribers
ros::Subscriber<std_msgs::Empty> subLED("toggle_led", &messageLEDCb);
// ros::Subscriber<control_msgs::JointJog> subPosCmd("joint_pos_cmd", &messagePosCmdCb );
ros::Subscriber<std_msgs::Float32> subPosCmdMotor1("joint_pos_cmd_motor_1", &messagePosCmdMotor1 );
ros::Subscriber<std_msgs::Float32> subPosCmdMotor2("joint_pos_cmd_motor_2", &messagePosCmdMotor2 );
ros::Subscriber<std_msgs::Float32> subPosCmdMotor3("joint_pos_cmd_motor_3", &messagePosCmdMotor3 );
ros::Subscriber<std_msgs::Float32> subPosCmdMotor4("joint_pos_cmd_motor_4", &messagePosCmdMotor4 );

///////////////////////////////////////////////////////////////////////////////////////////

// Function in the loop
void driveMotor(int PWM_PIN, int DIR_PIN, int SLP_PIN ,int PWM_CMD) {
	if (PWM_CMD > 0) {
		analogWrite(PWM_PIN, PWM_CMD);
		digitalWrite(DIR_PIN, HIGH);
		digitalWrite(SLP_PIN, HIGH);
	}

	if (PWM_CMD < 0) {
		PWM_CMD = abs(PWM_CMD);
		analogWrite(PWM_PIN, PWM_CMD);
		digitalWrite(DIR_PIN, LOW);
		digitalWrite(SLP_PIN, HIGH);
	}

	if (PWM_CMD == 0) {
		analogWrite(PWM_PIN, PWM_CMD);
		digitalWrite(DIR_PIN, LOW);
		digitalWrite(SLP_PIN, LOW);
	}
}

void driveAllMotors(double PWM_CMDS[4]){
	driveMotor(M1_PWM, M1_DIR, M1_SLP, PWM_CMDS[0]); 
	driveMotor(M2_PWM, M2_DIR, M2_SLP, PWM_CMDS[1]); 
	driveMotor(M3_PWM, M3_DIR, M3_SLP, PWM_CMDS[2]); 
	driveMotor(M4_PWM, M4_DIR, M4_SLP, PWM_CMDS[3]);
}

void initMotors() {
	pinMode(M1_PWM, OUTPUT);
	pinMode(M2_PWM, OUTPUT);
	pinMode(M3_PWM, OUTPUT);
	pinMode(M4_PWM, OUTPUT);

	pinMode(M1_DIR, OUTPUT);
	pinMode(M2_DIR, OUTPUT);
	pinMode(M3_DIR, OUTPUT);
	pinMode(M4_DIR, OUTPUT);

	pinMode(M1_SLP, OUTPUT);
	pinMode(M2_SLP, OUTPUT);
	pinMode(M3_SLP, OUTPUT);
	pinMode(M4_SLP, OUTPUT);

	// Idle all motors
	driveMotor(M1_PWM, M1_DIR, M1_SLP, 0); 
	driveMotor(M2_PWM, M2_DIR, M2_SLP, 0); 
	driveMotor(M3_PWM, M3_DIR, M3_SLP, 0); 
	driveMotor(M4_PWM, M4_DIR, M4_SLP, 0);
}

void initPIDs(){
	motor1_PID.SetSampleTime(INTERVAL_LOW_LVL_CTRL);
	motor2_PID.SetSampleTime(INTERVAL_LOW_LVL_CTRL);
	motor3_PID.SetSampleTime(INTERVAL_LOW_LVL_CTRL);
	motor4_PID.SetSampleTime(INTERVAL_LOW_LVL_CTRL);

	motor1_PID.SetOutputLimits(-255,255);
	motor2_PID.SetOutputLimits(-255,255);
	motor3_PID.SetOutputLimits(-255,255);
	motor4_PID.SetOutputLimits(-255,255);

	// Enable all PID controllers (AUTOMATIC/MANUAL)
	motor1_PID.SetMode(AUTOMATIC);
	motor2_PID.SetMode(AUTOMATIC);
	motor3_PID.SetMode(AUTOMATIC);
	motor4_PID.SetMode(AUTOMATIC);
}

void getJointFeedbacks(){
	motor_pos[0]  = (double) analogRead(M1_POT);
	motor_pos[1]  = (double) analogRead(M2_POT);
	motor_pos[2]  = (double) analogRead(M3_POT);
	motor_pos[3]  = (double) analogRead(M4_POT);

	for (int i = 0; i < 4; i++) {
    	motor_pos[i] = motor_pos[i]*motor_stroke_length/1024.0; // [mm]
    	joint_state.position[i] = (float) motor_pos[i];
    	// Show velocity as the des_joint_pos
    	des_joint_state.displacements[i] = (float) des_motor_pos[i]; 
    	joint_state.effort[i] = (float) motor_pwm[i];
    }
    // get the roscore timestamp
    joint_state.header.stamp = nh.now(); 
    des_joint_state.header.stamp = nh.now(); 

    // publish the joint states
    pubJointState.publish( &joint_state );
    pubDesJointState.publish( &des_joint_state );
}

void computePIDs(){
	motor1_PID.Compute();
	motor2_PID.Compute();
	motor3_PID.Compute();
	motor4_PID.Compute();
}

// Operation performing every fixed period
void tic(){
	getJointFeedbacks();
	computePIDs();
	driveAllMotors( motor_pwm );
	nh.spinOnce();
}

// Create time action for controlling the period
TimedAction jointStateAction = TimedAction(INTERVAL_LOW_LVL_CTRL, tic);

///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Put your setup code here, to run once:
	pinMode(LED_BUILTIN, OUTPUT);
	initMotors();
	initPIDs();

	nh.initNode();
	nh.subscribe(subLED);
	nh.subscribe(subPosCmdMotor1);
	nh.subscribe(subPosCmdMotor2);
	nh.subscribe(subPosCmdMotor3);
	nh.subscribe(subPosCmdMotor4);

	nh.advertise(pubJointState);
	nh.advertise(pubDesJointState);

	//Assign the initialized arrays to the message (http://wiki.ros.org/rosserial/Overview/Messages)
	joint_state.header.frame_id = "/joint_state";
	joint_state.name=name;
	joint_state.position=pos;
	joint_state.velocity=vel;
	joint_state.effort=eff;
  	/*Set the length To determine the end of the array each one has an auto-generated 
  	integer companion with the same name and the suffix_length*/
	joint_state.name_length=4;
	joint_state.position_length=4;
	joint_state.velocity_length=4;
	joint_state.effort_length=4;

	//Assign the initialized arrays to the message 
	des_joint_state.header.frame_id = "/des_joint_state";
	des_joint_state.joint_names=des_name;
	des_joint_state.displacements=des_pos;
	des_joint_state.velocities=des_vel;
	des_joint_state.duration=0;
  	// Set the length To determine the end of the array each one has an auto-generated 
  	// integer companion with the same name and the suffix_length
	des_joint_state.joint_names_length=4;
	des_joint_state.displacements_length=4;
	des_joint_state.velocities_length=4;

	// run all tasks in the loop once first time
	des_motor_pos[0] = 100;
	des_motor_pos[1] = 100;
	des_motor_pos[2] = 100;
	des_motor_pos[3] = 100;
	tic();
}

void loop() {
  	// Put your main code here, to run repeatedly:
	jointStateAction.check();
}
///////////////////////////////////////////////////////////////////////////////////////////
