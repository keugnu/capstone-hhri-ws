// System
#include <unistd.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/servo.h"
#include "std_msgs/UInt16MultiArray.h"


const int _SERVO_SPEED_ = 20;
const int _SERVO_POS_CHANGE_ = 20;

ros::NodeHandlePtr n = NULL;
int last_position = 90; // starting position at initialization is 90 degrees

/*  Function: track_callback
    desc:   Callback function for the IR sensor topic. Sends requests to the maestro service to move servos.
    inputs:
        &msg:   Array containing the current message on the IR sensor topic.
    outputs:
        servo_client.call: sends a request to the maestro service
*/
void track_callback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    ros::ServiceClient servo_client = n->serviceClient<hbs2::servo>("servo_srv");
    hbs2::servo srv_servo;

    int index = 0;

    // search through the topic queue and find the minimum
    for (int i = 1; i < msg->data.size(); i++) {
        if (msg->data.at(i) < msg->data.at(index))
            index = i;
    }

    switch(index) {
        case 0: {
            ROS_INFO("Object is centered.");
            int diff = 0;
            if (last_position == 90) { break; }
            else { 
                diff = 90 - last_position;
                if (diff > 0) {
                    srv_servo.request.command = 1;
                    srv_servo.request.position = last_position + _SERVO_POS_CHANGE_;
                    last_position += _SERVO_POS_CHANGE_;
                }
                else {
                    srv_servo.request.command = 1;
                    srv_servo.request.position = last_position - _SERVO_POS_CHANGE_;
                    last_position -= _SERVO_POS_CHANGE_;
                }
                servo_client.call(srv_servo);
            }
            break;
        }
        case 1: {
            ROS_INFO("Object is right of center.");
            if (last_position < 140) {
                // Make sure that the servo speed is set properly before sending the position request
                srv_servo.request.command = 2;
                srv_servo.request.speed = _SERVO_SPEED_;
                servo_client.call(srv_servo);
                srv_servo.request.command = 1;
                srv_servo.request.position = last_position + _SERVO_POS_CHANGE_;
                last_position += _SERVO_POS_CHANGE_;
                servo_client.call(srv_servo);
            }
            break;
        }
        case 2: {
            ROS_INFO("Object is left of center.");
            if (last_position > 40) {
                // Make sure that the servo speed is set properly before sending the position request
                srv_servo.request.command = 2;
                srv_servo.request.speed = _SERVO_SPEED_;
                servo_client.call(srv_servo);            
                srv_servo.request.command = 1;
                srv_servo.request.position = last_position - _SERVO_POS_CHANGE_;
                last_position -= _SERVO_POS_CHANGE_;
                servo_client.call(srv_servo); 
            }
            break;
        }
        default:
            break;
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
    ros::init(argc, argv, "track");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_track", 5, track_callback);

    ros::spin();

    return 0;
}
