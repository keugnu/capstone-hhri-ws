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

ros::NodeHandlePtr n = NULL;

// If a touch has occurred at the correct pin, the humanoid will say thank you
void touchCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("Touch occurred at pin %u", msg->data);
    
    // Call tts service with text ("Thank you")
    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    hbs2::tts srv_tts;

    if (msg->data == 1) {
        srv_tts.request.text = "Thank you";
        tts_client.call(srv_tts);
    }

    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "touch");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_touch", 5, touchCallback);

    ros::spin();

    return 0;
}
