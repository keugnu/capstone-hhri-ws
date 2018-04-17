// System
#include <stdlib.h>
#include <string>
#include <sstream>

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

/*	Function: convert_from_deg (Convert From Degrees)
	desc:	Converts a number from degrees to spatial position understandable by the maestro.
	inputs:
		deg:	degrees
	outputs:
		int:	spatial position for the maestro to turn a servo
*/
int convert_from_deg(int deg) {
	return 36*deg + 1060;
}

/*	Function: change_speed
	desc: sends a command to the maestro to change the speed of servo movement
	inputs:
		spd:	the speed that is desired
	outputs:
		bool: true if successful else false
*/
bool change_speed(int spd) {
	ROS_INFO("Changing servo speed to %i RPM.", spd);
	std::stringstream ss;
	ss << _MAESTRO_SPDCTRL_ << " " << _MAESTRO_DEV_ << " " << _MAESTRO_CHANNEL_ << " " << std::to_string(spd);
	std::string syscomspd = ss.str();
	if (system((syscomspd).c_str()) != 0) { return false; }
	else { return true; }
}

/*	Function: move
	desc: sends a command to the maestro to turn the servo motor
	inputs:
		pos:	the position (in degrees) that is desired
	outputs:
		bool: true if successful else false
*/
bool move(int pos) {
	ROS_INFO("Changing servo position to %i degrees.", pos);
	pos = convert_from_deg(pos);
	std::stringstream ss;
	ss <<  _MAESTRO_POSCTRL_ << " " << _MAESTRO_DEV_ << " " << _MAESTRO_CHANNEL_ << " " << std::to_string(pos);
	std::string syscompos = ss.str();
	if (system((syscompos).c_str()) != 0) { return false; }
	else { return true; }
}

/*	Function: handle_req (handle request)
	desc:	The callback function for the service request. Interprets the service input and
			sends commands via func:move or func:change_speed.
	inputs:
		&req:	service request content object
		&res:	service repsone content object
	outputs:
		&res:	service response content object
*/
bool handle_req(hbs2::servo::Request &req, hbs2::servo::Response &res) {
	switch (req.command) {
		case 1: {
			ROS_DEBUG("A request to move the position of the head has been made.");
			if(move(req.position)) { return true; }
			else {
				ROS_ERROR("The servo request to change position has failed.");
				return false;
			}
		}
		case 2: {
			ROS_DEBUG("A request to change the speed of head movement has been made.");
			if(change_speed(req.speed)) { return true; }
			else {
				ROS_ERROR("The servo request to change speed has failed.");
				return false;
			}
		}
		default:
			break;
	}
}

/*	Function: init_pos (initialize position)
	desc: sends a command to the maestro to change the position of servo(s) to center
	inputs:
		None
	outputs:
		bool: true if successful else false
*/
bool init_pos() {
	std::stringstream ss;
	ss << _MAESTRO_POSCTRL_ << " " << _MAESTRO_DEV_ << " " << _MAESTRO_CHANNEL_ << " " << std::to_string(4500);
	std::string syscompos = ss.str();
	if(system(syscompos.c_str()) != 0) {
		ROS_ERROR("Servo initialization to 90 degrees failed.");
		return false;
	}
	else {
		ROS_INFO("Servo initialization to 90 degrees complete.");
		return true;
	}
}

/*  Function: main
    desc: Entry point for the Node
    inputs:
        argc: count of command line arguments
        argv: array of command line arguments
    outputs:
        int: always 0 if exits gracefully
*/
int main(int argc, char** argv) {
	ros::init(argc, argv, "servo");
	ros::NodeHandle n;

	ros::ServiceServer srv = n.advertiseService("servo_srv", handle_req);
	ROS_INFO("ROS servo service has started.");
	
	if (init_pos()) { ros::spin(); }
	else { 
		ROS_FATAL("Failed to initialize servo. Exiting...");
		exit(1);
	}
	
	return 0;
}