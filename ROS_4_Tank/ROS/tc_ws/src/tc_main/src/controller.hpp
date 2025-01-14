
#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

#include "UART.hpp"

class SimpleAckermannSteeringController {
public:
	SimpleAckermannSteeringController(ros::NodeHandle& nh);
	~SimpleAckermannSteeringController();
	
protected:
	bool all_motors_sim;
	
	UART* uart;

	
	ros::NodeHandle nh;

	ros::Subscriber motors_en_sub;
	void motors_en_cb(const std_msgs::Bool::ConstPtr& msg);
	volatile bool motors_en;

	float speed;
	ros::Subscriber cmd_sub;
	void cmd_cb(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
	
	ros::Timer non_realtime_loop_timer;
	int64_t prev_pulse_cnt_fb;
	void publish_odom(const ros::TimerEvent& e);
	ros::Publisher odom_pub;
	nav_msgs::Odometry odom_msg;
	ros::Time last;
	float traversed_path;
};
