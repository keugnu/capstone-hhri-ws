// System
#include <stdlib.h>
#include <stdio.h>
#include <string>

// ROS
#include "ros/ros.h"
#include "hbs2/iot.h"
#include "hbs2/tts.h"
#include "hbs2/sonar.h"

// Declare a global NodeHandle
ros::NodeHandlePtr n = NULL;

/*  Function: handle_req (Handle request)
    desc: Callback function for the TTS service. Sends system call to TTS application, festival.
    inputs:
        &req: req.text: text to be sent to TTS application
    outputs:
        &res: response.success: boolean for successful system call
*/
bool handle_req(hbs2::tts::Request &req, hbs2::tts::Response &res)
{
    char buf[150];
    int str_len = sprintf(buf, "/bin/bash -c 'echo \"%s\" \u007C festival --tts'", req.text.c_str());
    if (!system(buf)) {
        ROS_ERROR("System call to festival failed.");
        return false;
    }
    else {
        ROS_INFO("System call to festival succeeded.");
        return true;
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
int main(int argc, char **argv)
{  
    ros::init(argc, argv, "tts_srv");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    
    ros::ServiceServer srv = n->advertiseService("tts_srv", handle_req);
    ROS_INFO("ROS TTS service has started.");
    ros::spin();

    return 0;
}
