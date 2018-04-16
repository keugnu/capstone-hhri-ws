/*      Touch behavior node, subscriber to MPR121 hardware node publisher       */
// System
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <iostream>


// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "std_msgs/UInt8.h"

// declare global NodeHandle
ros::NodeHandlePtr n = NULL;

/*  Function: touch_callback
    desc: If a touch has occurred at the correct pin, the humanoid will say thank you
    inputs:
        &msg: contains the current data on the tpc_touch queue
    outputs:
        tts_client.call: Sends a request to the TTS service
*/
bool touch_callback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_DEBUG("Touch occurred at pin %u", msg->data);

    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    hbs2::tts srv_tts;

    if (msg->data == 1 || msg->data == 3) {
        srv_tts.request.text = "Thank you";
        if(tts_client.call(srv_tts)) {
            ROS_INFO("TTS service call from touch behavior completed successfully.");
            return true;
        }
        else { ROS_ERROR("TTS service call from touch behavior failed."); }
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
    ros::init(argc, argv, "touch");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_touch", 5, touch_callback);

    ros::spin();

    return 0;
}
