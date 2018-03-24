// System
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <functional>

// ROS
#include "ros/ros.h"
#include "hbs2/iot.h"
#include "hbs2/sonar.h"
#include "hbs2/tts.h"
#include "hbs2/servo.h"


ros::NodeHandlePtr n = NULL;


bool handle_req(hbs2::iot::Request &req, hbs2::iot::Response &res) {
    ROS_INFO("Serving request from [iort] node.");
    
    ros::ServiceClient sonar_client = n->serviceClient<hbs2::sonar>("sonar_srv");
    ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
    ros::ServiceClient servo_client = n->serviceClient<hbs2::servo>("servo_srv");

    hbs2::sonar srv_sonar;
    hbs2::tts srv_tts;
    hbs2::servo srv_servo;

    switch (req.command) {
        case 1: {
            ROS_INFO("Serving request to read sonar sensor.");
            sonar_client.call(srv_sonar);
            if (srv_sonar.response.success == true) {
                res.success = true;
                res.data = srv_sonar.response.data;
            }
            else {
                ROS_ERROR("Request to read sonar sensor failed in iot_srv.");
                res.success = false;
                return false;
            }
            break;
        }
        case 2: {
            ROS_INFO("Serving request for TTS.");
            srv_tts.request.text = req.text;
            tts_client.call(srv_tts);
            if (srv_tts.response.success) { res.success = true; }
            else {
                ROS_ERROR("TTS Request failed in iot_srv.");
                res.success = false;
                return false;
            }
            break;
        }
        case 3: {
            ROS_INFO("Serving request to shake head.");
            srv_servo.request.command = 2;
            srv_servo.request.speed = 100;
            if(servo_client.call(srv_servo)) {
                ROS_INFO("Servo speed has changed to 100 RPM.");
            }
            else {
                ROS_ERROR("Request to change speed of the servo has failed.");
            }

            for(int i = 1; i < 5; i++) {
                srv_servo.request.command = 1;
                if (i % 2)
                    srv_servo.request.position = 60;
                else
                    srv_servo.request.position = 120;
                servo_client.call(srv_servo);
                usleep(200000);
            }

            srv_servo.request.position = 90;
            servo_client.call(srv_servo);

            if (srv_servo.response.success == true) { res.success = true; }
            else {
                ROS_ERROR("Request to shake head has failed in iot_srv.");
                res.success = false;
                return false;
            }
            break;
        }
    }
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "iot_srv");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::ServiceServer srv = n->advertiseService("iot_srv", handle_req);

    ROS_INFO("ROS IoT Service has started.");
    ros::Rate loop_rate(10);
    ros::spin();
    return 0;
}
