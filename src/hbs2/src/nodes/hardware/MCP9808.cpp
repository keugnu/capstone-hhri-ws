/* 	
	MCP9808 client node that calls upon i2c_mgr.cpp service
	req format: [type, device addr, reg1, dat1, <dat1>,...,regN, datN, <datN>]
	if type = read req, dat1:N = 0
*/

// System
#include <unistd.h>
#include <stdlib.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"
#include "hbs2/temp.h"

// declare global NodeHandle
ros::NodeHandlePtr n = NULL;

bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x00, 0x18, 0x00, 0x00};
    usleep(1000);
    if (client.call(srv)) { return false; }
    else { return true; }
}

// Configure sensor and set temperature resolution
bool write_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(5);
    srv.request.request = {0x02, 0x18, 0x01, 0x00, 0x00};
    srv.request.size = 5;
    srv.request.bus = 0;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));
        srv.request.request.resize(4);
        srv.request.request = {0x02, 0x18, 0x08, 0x03};
        srv.request.bus = 0;


        if (client.call(srv)) { 
            /* wait for job to be served in the i2c manager. */
            while(!status_req(client, srv));
            return true;
        }
    }
    else {
        ROS_ERROR("Failed to call service: i2c_srv.");
        return false;
    }
}

// Read temperature from register
float read_temp(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x01, 0x18, 0x05, 0x00};
    srv.request.bus = 0;
	
    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));

        int temp = (srv.response.data.at(0) & 0x1F) * 256 + srv.response.data.at(1);
        if (temp > 4095) { temp -= 8192; }

        return ((float)temp * 0.0625) * 1.8 + 32;
    }
    else {
        ROS_ERROR("Failed to call service: i2c_srv.");
        return 1;
    }
}

// Report temp
bool report_temp(hbs2::temp::Request &req, hbs2::temp::Response &res) {
    ros::ServiceClient client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;

    if (write_init(client, srv)) {
        float temperature = read_temp(client, srv);
        ROS_DEBUG("Temperature in Fahrenheit: %.2f\n", temperature);
        res.data = temperature;
        return true;
    }
    else {
        ROS_ERROR("Failed to read temperature.");
        return false;
    }
}


int main(int argc, char **argv) {

    // Initialize temp sensor node
    ros::init(argc, argv, "mcp9808");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::ServiceServer temp_srv = n->advertiseService("temp_srv", report_temp);
    ROS_INFO("ROS temperature service has started.");
    ros::spin();

    return 0;	
}
