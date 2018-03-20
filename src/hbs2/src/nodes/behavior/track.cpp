#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/UInt16.h"

void track_callback(const std_msgs::UInt16::ConstPtr& msg) {
    ROS_INFO("Distance: %umm", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "track");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tpc_track", 5, track_callback);

    ros::spin();

    return 0;
}
