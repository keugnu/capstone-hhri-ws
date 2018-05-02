/*	SRF02 sonar client node that calls upon i2c_mgr.cpp	*/

// System
#include <unistd.h>
#include <stdlib.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"
#include "hbs2/sonar.h"

// declare global NodeHandle
ros::NodeHandlePtr n = NULL;


bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x00, 0x71, 0x00, 0x00};

    usleep(1000);
    if (client.call(srv)) { return false; }
    else { return true; }
}


// Configure sensor and set range to centimeters
bool write_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x02, 0x71, 0x00, 0x51};
    srv.request.bus = 1;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));
	    return true;    
    }

    ROS_ERROR("Failed to call service i2c_srv.");
    return false;
}

/*  Function: read_range
    desc: Performs a request to the i2c bus manager service to read the device.
    inputs:
        &client: i2c bus manager service client object
        &srv: i2c bus manager request inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager
        uint16: two byte result which is the measurement received from the sensor
*/
uint16_t read_range(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    if (write_init(client, srv)) { ROS_INFO("SRF02 is prepared to be read."); }

    srv.request.request.resize(6);
    srv.request.request = {0x01, 0x71, 0x00, 0x00, 0x00, 0x00};
    srv.request.bus = 1;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        ROS_INFO("Request to i2c manager successful. Waiting to be served.");
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

/*  Function: report_range
    desc: Callback function for the sonar service. Sends a request to read a measurement from the sensor.
    inputs:
        &req: sonar service request inputs object
        argv: sonar service response outputs object
    outputs:
        bool: true if a measurement was received from the device sucessfully, else false
*/
bool report_range(hbs2::sonar::Request &req, hbs2::sonar::Response &res) {
    ros::ServiceClient i2c_client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv_i2c;
    usleep(1000);
    uint16_t range = read_range(i2c_client, srv_i2c);
    ROS_DEBUG("Range in centimeters: %u", range);
    if (range > 0) { res.data = range; return true; }
    else { return false; }
}

/*  Function: main
    desc: Entry point for the Node
    inputs:
        argc: count of command line arguments
        argv: array of command line arguments
    outputs:
        int: always 0 if exits gracefully
*/
int main(int argc, char **argv) {
    // Initialize sonar sensor node
    ros::init(argc, argv, "srf02");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::ServiceClient client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv_i2c;    

    ros::ServiceServer srv = n->advertiseService("sonar_srv", report_range);
    ROS_INFO("ROS sonar service has started.");
    ros::spin();

    return 0;	
}
