/*      Wave gesture behavior node, subscriber to ZX_Sensor hardware node publisher       */
// System
#include <unistd.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "std_msgs/UInt8.h"

ros::NodeHandlePtr n = NULL;

/*  Function: wave_callback
    desc: If a wave gesture has occurred, the humanoid will say hello
    inputs:
        &msg: contains the current data on the tpc_wave queue
    outputs:
        tts_client.call: sends a service request to the TTS service
*/
void wave_callback(const std_msgs::UInt8::ConstPtr& msg) {
    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    hbs2::tts srv_tts;

    if (msg->data == 1) {
        srv_tts.request.text = "Hello";
        tts_client.call(srv_tts);
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
    ros::init(argc, argv, "wave");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_wave", 5, wave_callback);

    ros::spin();

    return 0;
}
