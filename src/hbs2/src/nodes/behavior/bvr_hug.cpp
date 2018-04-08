// System
#include <unistd.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include "hbs2/tts.h"
#include "std_msgs/Int16MultiArray.h"

ros::NodeHandlePtr n = NULL;

void adc_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    // Call tts service with text ("I love you") if hug occurs

    if ((msg->data.at(3) * 0.0001875) > 2.0 && (msg->data.at(2) * 0.0001875) > 2.0) {
        ros::ServiceClient tts_client = n->serviceClient<hbs2::tts>("tts_srv");
        hbs2::tts srv_tts;
        ROS_INFO("A hug has occurred.");
        srv_tts.request.text = "I love you";
        tts_client.call(srv_tts);
        // Block 2 seconds before sensing a hug again
        usleep(2000000);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hug");
    n = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Subscriber sub = n->subscribe("tpc_adc", 5, adc_callback);

    ros::spin();

    return 0;
}