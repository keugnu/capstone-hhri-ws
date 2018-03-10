/*      Touch behavior node, subscriber to MPR121 hardware node publisher       */

#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/UInt8.h"

void touchCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("Touch occurred at pin %u", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "touch");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("tpc_touch", 5, touchCallback);

    ros::spin();

    return 0;
}
