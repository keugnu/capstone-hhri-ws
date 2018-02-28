// System
#include <sys/types.h>
#include <string>
#include <queue>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"

// Custom
#include "Request.hpp"


int fd;
char const* iic_dev = "/dev/i2c-1";
std::queue<Request> work_queue, completed_queue;


bool write_req(Request* job) {
    if (fd = open(iic_dev, O_RDWR) < 0) { return false; }
    else {
	char to_write[job->data.size()] = {0};
	char dev_addr = job->get_id();
	for (int i = 0; i < job->data.size(); i++) {
	    to_write[i] = job->data.at(i);
	}

	ioctl(fd, I2C_SLAVE, dev_addr);

	if (write(fd, to_write, job->data.size()) != job->data.size()) { return false; }
    }

    job->set_status(true);
    return true;
}

bool read_req(Request* job) {
    if (fd = open(iic_dev, O_RDWR) < 0) { return false; }
    else {
	char is_read[job->data.size()] = {0};
        char dev_addr = job->get_id();
        for (int i = 0; i < job->data.size(); i++) {
            is_read[i] = job->data.at(i);
        }

        ioctl(fd, I2C_SLAVE, dev_addr);

	if (read(fd, is_read, job->data.size()) != job->data.size()) { 
	    return false; 
	} else {
	    job->data.resize(job->data.size());
    	    memcpy(&job->data[0], &is_read[0], job->data.size() * sizeof(char));
	    job->set_status(true);
	    return true;
	}

    }
}

bool handle_req(hbs2::i2c_bus::Request &req, hbs2::i2c_bus::Response &res) {
    Request request = Request(req.request, req.size);

    if (request.get_type() == "status") {
	res.success = false;
	for (int i = 0; i < completed_queue.size(); i++) {
	    Request job = completed_queue.front();
	    if (job.get_id() == req.request[1]) { res.success = true; break; }
	    else { completed_queue.pop(); completed_queue.push(job); }    
	}
	completed_queue.pop();
	return true;
    }
    else { work_queue.push(request); }
    
    if (!work_queue.empty()) {

	// TODO sort on priority

	Request job = work_queue.front();
	work_queue.pop();
	if (job.get_type() == "read") { 
	    if (read_req(&job)) { completed_queue.push(job); res.data = job.data; }
	    else { work_queue.push(job); }
	}
	else if (job.get_type() == "write") { 
	    if (write_req(&job)) { completed_queue.push(job); }
	    else { work_queue.push(job); }

    	}
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "i2c_bus_mgr_srv");
    ros::NodeHandle n;

    ros::ServiceServer srv = n.advertiseService("i2c_srv", handle_req);
    ros::spin();        
    return 0;
}
