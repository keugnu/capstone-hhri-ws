// System
#include <stdlib.h>
#include <stdio.h>
#include <string>

// ROS
#include "ros/ros.h"
#include "hbs2/iot.h"
#include "hbs2/tts.h"
#include "hbs2/sonar.h"

ros::NodeHandlePtr n = NULL;


bool handle_req(hbs2::tts::Request &req, hbs2::tts::Response &res)
{
    char buf[150];
    //std::string buf = "echo " + req.text + " | " + "festival --tts";
    int str_len = sprintf(buf, "/bin/bash -c 'echo \"%s\" \u007C festival --tts'", req.text.c_str());
    if (system(buf) != 0) {
        ROS_ERROR("System call to festival failed.");
        res.success = false;
        return false;
    }
    else {
        res.success = true;
        ROS_INFO("System call to festival succeeded.");
        return true;
    }  
}


int main(int argc, char **argv)
{  
    // Initial tts node  
    ros::init(argc, argv, "tts_srv");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    
    ros::ServiceServer srv = n->advertiseService("tts_srv", handle_req);
    ROS_INFO("ROS TTS service has started.");
    ros::spin();

    // End program
    return 0;
}
