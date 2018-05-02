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

/*  Function: status_req (status request)
    desc: Sends a request to the i2c bus manager for the status of the current read or
          write request for a device.
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the previous request is complete, o/w false
*/
bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x00, 0x18, 0x00, 0x00};
    usleep(1000);
    if (client.call(srv)) { return false; }
    else { return true; }
}

/*  Function: write_init (write initialization)
    desc: Initializes the sensor by configuring the sensor and setting temperature resolution
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the i2c bus requests complete successfully, o/w false
*/
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

/*  Function: read_temp (read temperature)
    desc: Reads temperature from the sensor
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        float: the current ambient temperature reading from the sensor in celsius, o/w 1.0
*/
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

/*  Function: report_temp (report temperature)
    desc: Calls read_temp and
    inputs:
        &req: temperature service request input object
        &res: temperature service response output object
    outputs:
        bool: true of the service call was successful and a measurement has been received from the device


    WARNING:    THIS FUNCTION MAY NOT BE NEEDED. WRITE_INIT SHOULD NOT BE CALLED IN THIS FUNCTION.
                PART OF THIS FUNCTION IS LEFT OVER FROM WHEN THIS NODE WAS A PUBLISHER.
*/
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

/*  Function: main
    desc: Entry point for the Node
    inputs:
        argc: count of command line arguments
        argv: array of command line arguments
    outputs:
        int: always 0 if exits gracefully
*/
int main(int argc, char **argv) {

    // Initialize temp sensor node
    ros::init(argc, argv, "mcp9808");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::ServiceServer temp_srv = n->advertiseService("temp_srv", report_temp);
    ROS_INFO("ROS temperature service has started.");
    ros::spin();

    return 0;	
}
