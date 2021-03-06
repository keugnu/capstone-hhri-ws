/*      MPR121 Touch Sensor Hardware Node       */

// System
#include <unistd.h>
#include <stdint.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"
#include "std_msgs/UInt8.h"


/*  Function: status_req (status request)
    desc: Sends a request to the i2c bus manager for the status of the current read or
          write request for a device.
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
        address: i2c device address making the request
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the previous request is complete, o/w false
*/
bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x00, 0x5A, 0x00, 0x00};
    usleep(1000);
    if (client.call(srv)) { return false; }
    else { return true; }
}

/*  Function: write_init (write initialization)
    desc: Performs soft reset and start with MPR121 in "Stop mode" to prevent random reads.
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the i2c bus requests complete successfully, o/w false
*/
bool write_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.request = {0x02, 0x5A, (uint8_t)0x80, 0x63};
    srv.request.bus = 1;

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));
        srv.request.request.resize(4);
        srv.request.request = {0x02, 0x5A, 0x5E, 0x00};
        srv.request.bus = 1;

        if (client.call(srv)) {
            /* wait for job to be served in the i2c manager. */
            while(!status_req(client, srv));            
            return true;
        }
        else {
            ROS_ERROR("Failed to call i2c_srv: [MPR121 write_init]");
            return false;
        }
    }
    else { 
        ROS_ERROR("Failed to call i2c_srv: [MPR121 write_init]");
        return false;
    }
}

/*  Function: touch_init (touch initialization)
    desc: Configures the touch and release threshold registers.
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the i2c bus requests complete successfully, o/w false
*/
bool touch_init(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    // Touch and release threshold registers 
    uint16_t thres_reg = 0x41;
    uint16_t rel_reg = 0x42;

    // Touch and release thresholds
    uint16_t touch_thres = 12;
    uint16_t rel_thres = 6;

    srv.request.request.resize(4);
    srv.request.bus = 1;

    // Set up 13 touch channels with touch threshold = 12
    srv.request.request = {0x02, 0x5A, thres_reg, touch_thres};

    for (int i = 0; i < 12; i++) {
        if (client.call(srv)) {
            /* wait for job to be served in the i2c manager. */
            while(!status_req(client, srv));            
            thres_reg += i*2;
            srv.request.request = {0x02, 0x5A, thres_reg, touch_thres};
        }
        else {
            ROS_ERROR("Failed to call i2c_srv: [touch_init]");
            return false;
        }
    }

    // Set up 13 touch channels with release threshold = 6
    srv.request.request = {0x02, 0x5A, rel_reg, rel_thres};

    for (int i = 0; i < 12; i++) {
        if (client.call(srv)) {
            /* wait for job to be served in the i2c manager. */
            while(!status_req(client, srv));            
            rel_reg += i*2;
            srv.request.request = {0x02, 0x5A, rel_reg, rel_thres};
        }
        else {
            ROS_ERROR("Failed to call i2c_srv: [touch_init]");
            return false;
        }
    }
    return true;
}

/*  Function: reg_init (registers initialization)
    desc: Set up registers (MHD, NHD, NCL, FDL, Debounce touch & release, Baseline tracking)
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        bool: true if the status of the i2c bus requests complete successfully, o/w false
*/
bool reg_setup(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    srv.request.request.resize(4);
    srv.request.bus = 1;
    srv.request.request = {0x02, 0x5A, 0x00, 0x00};

    // MHD rising reg 0x2B, NHD rising reg 0x2C, NCL rising reg 0x2D, FDL rising reg 0x2E
    srv.request.request[2] = 0x2B; srv.request.request[3] = 0x01;
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x2C; srv.request.request[3] = 0x01; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }    
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x2D; srv.request.request[3] = 0x0E; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x2E; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }

    // MHD falling reg, 0x2F, NHD falling reg 0x30, NCL falling reg 0x31, FDL falling reg 0x32
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x2F; srv.request.request[3] = 0x01; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x30; srv.request.request[3] = 0x05; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x31; srv.request.request[3] = 0x01; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x32; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }

    // NHD touched reg 0x33, NCL touched 0x34, FDL touched 0x35
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x33; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x34; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x35; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }

    // Debounce touch & release reg 0x5B, config 1 reg 0x5c, config 2 ref 0x5D
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x5B; srv.request.request[3] = 0x00; 
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x5C; srv.request.request[3] = 0x10; // 16uA current
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    srv.request.request[0] = 0x02; srv.request.request[2] = 0x5D; srv.request.request[3] = 0x20; // 0.5 us encoding
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }

    srv.request.request[0] = 0x02; srv.request.request[2] = 0x5E; srv.request.request[3] = 0x8F;
    if (!client.call(srv)) { return false; }
    else {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));        
    }
    
    return true;
} 

/*  Function: report_touch
    desc: Reads for touches on the device.
    inputs:
        &client: i2c bus manager client object
        &srv: i2c bus manager service inputs object
    outputs:
        client.call: Sends a request to the i2c bus manager service
        uint8: Pin number which has registered a touch, o/w 255
*/
uint8_t report_touch(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint16_t wasTouched = 0x0000, readTouch[2] = {0x0000}, currentlyTouched = 0;
    srv.request.request.resize(6);
    srv.request.bus = 1;

    // Touch status register = 0x00
    srv.request.request = {0x01, 0x5A, 0x00, 0x00, 0x00, 0x00};
    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv));
         
        readTouch[0] = ((uint16_t)srv.response.data.at(1)) << 8;
        readTouch[0] |= ((uint16_t)srv.response.data.at(0));
        readTouch[1] = ((uint16_t)srv.response.data.at(3)) << 8;
        readTouch[1] |= ((uint16_t)srv.response.data.at(2));

        currentlyTouched = readTouch[0];
        currentlyTouched |= readTouch[1] << 8;
        currentlyTouched &= 0x0FFF;

        for(int i = 0; i < 12; i++) {
            //
            //  WARNING:    PIN 4 IS ALWAYS REGISTERING A TOUCH
            //
            if ((currentlyTouched & (1 << i)) && !(wasTouched & (1 << i)) && (i != 4)) {
                if (i == 1 || i == 3) { ROS_DEBUG("Pin %i was touched.", i); }
                return i; 
            }
            if (!(currentlyTouched & (1 << i)) && (wasTouched & (1 << i))) {
                ROS_DEBUG("Pin %i was released.", i);
                return i;
            }
        }
    
    }

    return 255;
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
    // Initialize touch sensor node
    ros::init(argc, argv, "mpr121");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;
    // Create publisher:
    ros::Publisher touch_pub = n.advertise<std_msgs::UInt8>("tpc_touch", 5);

    // Running at 5Hz:
    ros::Rate loop_rate(5);

    if (write_init(client, srv)) {
        if (touch_init(client, srv)) {
            if (reg_setup(client, srv)) {
                ROS_INFO("MPR121 initialized successfully.");
                while (ros::ok()) {
                    // Stuff message object with data and then publish
                    std_msgs::UInt8 msg;
                    msg.data = report_touch(client, srv);
                        
                    // Broadcast message to anyone connected:
                    touch_pub.publish(msg);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            else { ROS_ERROR("MPR121 [reg_setup] failed."); }
        }
        else { ROS_ERROR("MPR121 [write_init] failed."); }
    }

    return 0;
}
