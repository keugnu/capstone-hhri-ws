/*	SRF02 sonar client node that calls upon i2c_mgr.cpp	*/

// System
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <vector>
#include <iostream>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"
#include "hbs2/sonar.h"


ros::NodeHandlePtr n = NULL;


bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.request = {0x00, 0x70, 0x00, 0x00};
    usleep(1000);
    client.call(srv);
    if(!srv.response.success) return false;
    else return true;
}


// Configure sensor and set range to centimeters
bool write_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x02, 0x70, 0x00, 0x51};
    srv.request.size = 4;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));
	return true;    
    }

    ROS_ERROR("Failed to call service i2c_srv");
    return false;
}

// Read range from register (result is 4 bytes)
uint16_t read_range(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    if(write_init(client, srv)) {
        ROS_INFO("SRF02 is prepared to be read.");
    }

    srv.request.request.resize(6);
    srv.request.request = {0x01, 0x70, 0x00, 0x00, 0x00, 0x00};
    srv.request.size = 6;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));

	uint16_t range = (uint16_t)srv.response.data.at(2);
	range <<= 8;
	range |= (uint16_t)srv.response.data.at(3);
	return range;
    }
    else {
        ROS_ERROR("Failed to call service i2c_srv");
        return 1;
    }
}


bool report_range(hbs2::sonar::Request &req, hbs2::sonar::Response &res) {
    ros::ServiceClient i2c_client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv_i2c;
    usleep(1000);
    uint16_t range = read_range(i2c_client, srv_i2c);
    res.data = range;
    res.success = true;
    ROS_INFO("Range in centimeters: %u", range);
    return true;
}


int main(int argc, char **argv) {

    // Initialize sonar sensor node
    ros::init(argc, argv, "srf02");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ROS_INFO("The sonar sensor is being initialized.");
    ros::ServiceClient client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv_i2c;

    /*
    while(true) {
        uint16_t range = read_range(client, srv_i2c);
        ROS_WARN("Range in cm: %u", range);
    }
    */
    

    ros::ServiceServer srv = n->advertiseService("sonar_srv", report_range);
    ROS_INFO("ROS sonar service has started.");
    ros::spin();

    return 0;	
}
