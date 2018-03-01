/* 	
	MCP9808 client node that calls upon i2c_mgr.cpp service
	req format: [type, device addr, reg1, dat1, <dat1>,...,regN, datN, <datN>]
	if type = read req, dat1:N = 0
*/


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <vector>

#include "ros/ros.h"
#include "hbs2/i2c_bus.h"

// Configure sensor and set temperature resolution
bool write_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(5);
    srv.request.request = {0x02, 0x18, 0x01, 0x00, 0x00};
    srv.request.size = 5;

    if (client.call(srv)) {
	srv.request.request.resize(4);
	srv.request.request = {0x02, 0x18, 0x08, 0x03};
	srv.request.size = 4;

	if (client.call(srv))
	    return true;    
    }

    ROS_ERROR("Failed to call service i2c_srv");
    return false;
}

// Read temperature from register
float read_temp(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(3);
    srv.request.request = {0x02, 0x18, 0x05};
    srv.request.size = 3;

    if (client.call(srv)) {
	srv.request.request.resize(4);
	srv.request.request = {0x01, 0x18, 0x00, 0x00};
	srv.request.size = 4;
	
	if (client.call(srv)) {
	    std::vector<signed char> data(srv.response.data);
	    int temp = (data[0] & 0x1F) * 256 + data[1];
	    if (temp > 4095) { temp -= 8192; }
	    float fTemp = (temp * 0.0625) * 1.8 + 32;

	    return fTemp;
	}
    }
    ROS_ERROR("Failed to call service i2c_srv");
    return -1;
}


int main(int argc, char **argv) {

    // Initialize temp sensor node
    ros::init(argc, argv, "mcp9808");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;

    if (write_init(client, srv)) {
	float temperature = read_temp(client, srv);
	ROS_INFO("Temperature in Fahrenheit: %.2f\n", temperature);
    }

    return 0;	
}
