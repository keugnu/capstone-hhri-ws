// System
#include <unistd.h>
#include <vector>
#include <iostream>

// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "hbs2/led.h"
#include "std_msgs/Int16MultiArray.h"


// declare a global NodeHandler
ros::NodeHandlePtr n = NULL;


/*  Function adc_callback
    desc:   Callback function called when new data exists on tpc_adc. If a hug is detected then sends a request
            to the TTS service
    inputs:
        &msg: current data in the tpc_adc queue
    outputs:
        tts_client.call: sends a request to the tts service
*/
void adc_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    if ((msg->data.at(0) * 0.0001875) < 2.8 && (msg->data.at(1) * 0.0001875) < 2.8) {
        ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
        hbs2::tts srv_tts;
        ros::ServiceClient led_client = n->serviceClient<hbs2::led>("led_srv");
        hbs2::led srv_led;
        ROS_INFO("A hug has occurred.");
        srv_tts.request.text = "I love you";
        if (tts_client.call(srv_tts)) { ROS_INFO("TTS service call from hug behavior completed successfully."); }
        else { ROS_ERROR("TTS service call from hug behavior failed."); }
        
        srv_led.request.color = 6;
        if (led_client.call(srv_led)) { ROS_INFO("LED service call from hug behavior completed successfully."); }
        else { ROS_ERROR("LED service call from hug behavior failed."); }

        sleep(1);   // wait to reset LEDs

        srv_led.request.color = 0;
        if (led_client.call(srv_led)) { ROS_INFO("LED service call from hug behavior completed successfully."); }
        else { ROS_ERROR("LED service call from hug behavior failed."); }
        
        // Block 2 seconds before sensing a hug again
        usleep(2000000);
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
    ros::init(argc, argv, "hug");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_adc", 5, adc_callback);

    ros::spin();

    return 0;
}