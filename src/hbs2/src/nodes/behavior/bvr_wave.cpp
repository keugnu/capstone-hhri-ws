/*      Wave gesture behavior node, subscriber to ZX_Sensor hardware node publisher       */
// System
#include <unistd.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "std_msgs/UInt8.h"

ros::NodeHandlePtr n = NULL;

// If a wave gesture has occurred, the humanoid will say hello
void waveCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("Wave!");
    
    // Call tts service with text ("Hello")
    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    hbs2::tts srv_tts;

    if (msg->data == 1) {
        srv_tts.request.text = "Hello";
        tts_client.call(srv_tts);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wave");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_wave", 5, waveCallback);

    ros::spin();

    return 0;
}
