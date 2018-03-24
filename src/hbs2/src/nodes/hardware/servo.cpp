
// System
#include <stdlib.h>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream>

// ROS
#include "ros/ros.h"
#include "hbs2/servo.h"


const int _MAESTRO_CHANNEL_ = 5;
const std::string _MAESTRO_DEV_ = "/dev/ttyACM0";
const std::string _MAESTRO_POSCTRL_ = "/home/nvidia/hbs2_1/src/hbs2/lib/maestro_set_pos.sh";
const std::string _MAESTRO_SPDCTRL_ = "/home/nvidia/hbs2_1/src/hbs2/lib/maestro_set_spd.sh";


/* COMMANDS		1: change position		values: 40-140 Degrees
				2: change speed			values: 1-100 RPM
*/


int convert_from_deg(int deg)
{
	int pos = 36*deg + 1060;
	return pos;
}


bool change_speed(int spd)
{
	ROS_INFO("Changing speed to %i RPM.", spd);
	std::stringstream ss;
	ss << _MAESTRO_SPDCTRL_ << " " << _MAESTRO_DEV_ << " " << _MAESTRO_CHANNEL_ << " " << std::to_string(spd);
	std::string syscomspd = ss.str();
	ROS_WARN("%s", syscomspd.c_str());
	if (system((syscomspd).c_str()) != 0) return false;
	else return true;
}


bool move(int pos)
{
	ROS_INFO("Changing head position to %i degrees.", pos);
	pos = convert_from_deg(pos);
	std::stringstream ss;
	ss <<  _MAESTRO_POSCTRL_ << " " << _MAESTRO_DEV_ << " " << _MAESTRO_CHANNEL_ << " " << std::to_string(pos);
	std::string syscompos = ss.str();
	ROS_WARN("%s", syscompos.c_str());
	if (system((syscompos).c_str()) != 0) return false;
	else return true;
}


bool handle_req(hbs2::servo::Request &req, hbs2::servo::Response &res)
{
	switch (req.command) {
		case 1: {
			ROS_INFO("A request to move the position of the head has been made.");
			if(move(req.position)) {
				res.success = true;
				return true;
			}
			else {
				ROS_ERROR("The position request has failed.");
				res.success = false;
				return false;
			}
			break;
		}
		case 2: {
			ROS_INFO("A request to change the speed of head movement has been made.");
			if(change_speed(req.speed)) {
				res.success = true;
				return true;
			}
			else {
				ROS_ERROR("The servo request has failed.");
				res.success = false;
				return false;
			}
			break;
		}
		default:
			break;
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "servo");
	ros::NodeHandle n;

	ros::ServiceServer srv = n.advertiseService("servo_srv", handle_req);
	ROS_INFO("ROS servo service has started.");
	
	ros::spin();
	
	return 0;
}