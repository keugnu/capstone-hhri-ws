#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"

void track_callback(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    int i = 0;
    for(std::vector<uint16_t>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
        ROS_INFO("IR sensor %i: %umm", (i+1), *it);
        i++;
    }

//    ROS_INFO("Distance: %umm", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "track");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tpc_track", 5, track_callback);

    ros::spin();

    return 0;
}
