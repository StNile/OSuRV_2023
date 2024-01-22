
#include "controller.hpp"

// Config.
#define LOOP_HZ 25


#define rad2deg(rad) (180/M_PI*(rad))
#define deg2rad(deg) (M_PI/180*(deg))
#if WIDE_PWM
// -pi/2=10%=100 +pi/2=90%=900
#define rad2duty(rad) (((rad) + M_PI_2)*(800/M_PI) + 100)
#define duty2rad(duty) (((duty) - 100)*(M_PI/800) - M_PI_2)
#else
// -pi/2=2.5%=25 +pi/2=12.5%=125
#define rad2duty(rad) (((rad) + M_PI_2)*(100/M_PI) + 25)
#define duty2rad(duty) (((duty) - 25)*(M_PI/100) - M_PI_2)
#endif


// Includes for driver.
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <sys/ioctl.h> // ioctl()


#include <algorithm>
using namespace std;

enum t_turn {TURN_LEFT, TURN_NONE, TURN_RIGHT};
enum t_move {MOVE_FORWARD, MOVE_NONE, MOVE_REVERSE};

#include <angles/angles.h>

#define DEBUG(x) do{ ROS_DEBUG_STREAM(#x << " = " << x); }while(0)

template<typename T>
static T sym_clamp(T x, T limit) {
	return clamp(x, -limit, limit);
}

SimpleAckermannSteeringController::SimpleAckermannSteeringController(ros::NodeHandle& nh) :
	nh(nh),
	all_motors_sim(false),
	uart(nullptr),
	motors_en(true)
{
	
	
	ros::param::param("~all_motors_sim", all_motors_sim, false);
	ROS_INFO(
		"all_motors_sim = %s",
		all_motors_sim ? "true" : "false"
	);
	
	
	if(!all_motors_sim){
		// Open driver.
		uart = new UART("/dev/serial0", 115200);

	}
	
	motors_en_sub = nh.subscribe(
		"motors_en",
		1,
		&SimpleAckermannSteeringController::motors_en_cb,
		this
	);
	
	cmd_sub = nh.subscribe(
		"cmd",
		1,
		&SimpleAckermannSteeringController::cmd_cb,
		this
	);
	
	
	ros::Duration timer_period = ros::Duration(1.0/LOOP_HZ);
	non_realtime_loop_timer = nh.createTimer(
		timer_period,
		&SimpleAckermannSteeringController::publish_odom,
		this
	);
	
	odom_pub  = nh.advertise<nav_msgs::Odometry>(
		"odom",
		1
	);
	
	odom_msg.header.frame_id = "bldc_enc";
	last = ros::Time::now();
}

SimpleAckermannSteeringController::~SimpleAckermannSteeringController() {
	if(!all_motors_sim){
		// Close driver.
		delete uart;
	}
}


void SimpleAckermannSteeringController::publish_odom(const ros::TimerEvent& e) {
	ros::Time now = ros::Time::now();
	
/*	
	odom_msg.header.seq++;
	odom_msg.header.stamp = now;
	odom_msg.pose.pose.position.x = traversed_path;
	
	odom_pub.publish(odom_msg);
	*/
	
}

void SimpleAckermannSteeringController::motors_en_cb(
	const std_msgs::Bool::ConstPtr& msg
) {
	motors_en = msg->data;
	ROS_INFO(
		"Motors %s", motors_en ? "enabled" : "disabled"
	);
}


void SimpleAckermannSteeringController::cmd_cb(
	const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg
) {
	//TODO Limits roc:
	// steering_angle_velocity
	// acceleration
	speed = msg->drive.speed;
	float steering_angle = msg->drive.steering_angle;

	// Limit backward speed to 90%.
	if(speed < -90) {
		speed = -90;
	}
	
#if 0
		ROS_DEBUG("speed = %f %%", speed);
		ROS_DEBUG(
			"steering_angle = %f rad (%f deg)",
			steering_angle,
			rad2deg(steering_angle)
		);
#endif
	
	if(!all_motors_sim && motors_en){
		

		speed = 100;//TODO Hardocoded.

		t_turn turn = TURN_NONE;
		if(steering_angle > M_PI/6){
			turn = TURN_RIGHT;
		}else if(steering_angle < -M_PI/6){
			turn = TURN_LEFT;
		}

		t_move move = MOVE_NONE;
		if(speed >= 50){
			move = MOVE_FORWARD;
		}else if(speed <= -50){
			move = MOVE_REVERSE;
		}

		uint8_t chassis_speed = 3;
		
		vector<uint8_t> cmd;
		uint8_t chassis_move;
		switch(move){
			case MOVE_NONE:
				chassis_move = 0;
				break;
			case MOVE_FORWARD:
				chassis_move = 2;
				break;
			case MOVE_REVERSE:
				chassis_move = 3;
				break;
		}
		uint8_t chassis_turn;
		switch(turn){
			case TURN_NONE:
				chassis_turn = 0;
				break;
			case TURN_RIGHT:
				chassis_turn = 1;
				break;
			case TURN_LEFT:
				chassis_turn = 2;
				break;
		}
		uint8_t chassis = (chassis_speed << 6) | (chassis_turn << 2) | chassis_move;
#if 1
		ROS_DEBUG("chassis_move = 0x%02x", chassis_move);
		ROS_DEBUG("chassis_turn = 0x%02x", chassis_turn);
		ROS_DEBUG("chassis_speed = 0x%02x", chassis_speed);
		ROS_DEBUG("chassis = 0x%02x", chassis);
#endif
		cmd.push_back(chassis);
		uart->write(cmd);
		
	}
}
