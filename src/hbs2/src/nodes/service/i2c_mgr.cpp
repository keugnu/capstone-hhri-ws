// System
#include <sys/types.h>
#include <string>
#include <vector>
#include <queue>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"

int fd;
std::string iic_dev = "/dev/i2c-1";
std::queue work_queue;


bool write_req(const std::vector<signed char>* req, const std::vector<signed char>* res) {

    return true;
}

bool read_req(const std::vector<signed char>* req, const std::vector<signed char>* res) {

    return true;
}

bool handle_req(hbs2::i2c_bus::Request &req, hbs2::i2c_bus::Response &res) {
    if (req.request[0] == 0) { 
        if (read_req(&req.request, &res.data)) {
	    ROS_INFO("Read request to I2C bus (address: %d) succeeded.", req.request[1]);
	    res.success = true;
	}
        else {
	    ROS_INFO("Read request to I2C bus (address: %d) failed.", req.request[1]);
	    res.success = false;
	}
    }

    else if (req.request[0] == 1) {
        if (write_req(&req.request, &res.data)) {
	    ROS_INFO("Write request to I2C bus (address: %d) succeeded.", req.request[1]);
	    res.success = true;
	}
        else { 
	    ROS_INFO("Write request to I2C bus (address: %d) failed.", req.request[1]);
	    res.success = false;
	}
    }

    else { 
        ROS_INFO("Request to I2C bus (type: %d) is not valid.", req.request[0]);
	res.success = false;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "i2c_bus_mgr_srv");
    ros::NodeHandle n;

    // TODO implement priority queue

    ros::ServiceServer srv = n.advertiseService("i2c_srv", handle_req);
    ros::spin();        
    return 0;
}
