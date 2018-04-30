/********************************************/
/*  Hardware node for LED Driver PCA9685    */
/*                                          */
/*  red = pwm.setPWM(0,0,2048)              */
/*  green = pwm.setPWM(1,0,2048)            */
/*  blue = pwm.setPWM(2, 0, 2048);          */
/*  red+blue = purple, red+green = yellow   */
/********************************************/

// System
#include <unistd.h>
#include <math.h>

// Custom
#include "PCA9685.h"

// ROS
#include "hbs2/led.h"

// declare global NodeHandle
ros::NodeHandlePtr n = NULL;


PCA9685::PCA9685(uint8_t addr) {
    _i2caddr = addr;
}


void PCA9685::begin(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    reset(client, srv);
    setPWMFreq(client, srv, 1000);
}


void PCA9685::reset(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    write8(client, srv, PCA9685_MODE1, 0x80);
    usleep(10);
}


void PCA9685::setPWMFreq(ros::ServiceClient &client, hbs2::i2c_bus &srv, float freq) {
    freq *= 0.9;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;

    uint8_t prescale = floor(prescaleval + 0.5);
    
    uint8_t oldmode = read8(client, srv, PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; //sleep
    write8(client, srv, PCA9685_MODE1, newmode);            //go to sleep
    write8(client, srv, PCA9685_MODE1, prescale);           //set the prescaler
    write8(client, srv, PCA9685_MODE1, oldmode);

    usleep(5);

    write8(client, srv, PCA9685_MODE1, oldmode | 0xa0);   /*This sets the MODE1 register to turn on auto increment; */
}


bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t address) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.bus = 1;
    srv.request.request = {0x00, address, 0x00, 0x00};
    usleep(1000);
    if (client.call(srv)) { return false; }
    else { return true; }
}


void PCA9685::setPWM(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t num, uint16_t on, uint16_t off) {
    srv.request.request.resize(7);
    srv.request.size = 7;
    srv.request.bus = 1;
    srv.request.request = {0x02, _i2caddr, (uint8_t)(LED0_ON_L + 4*num), (uint8_t)on, (uint8_t)(on>>8), (uint8_t)off, (uint8_t)(off>>8)};

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while (!status_req(client, srv, _i2caddr));
    }
    else { ROS_ERROR("Unable to write to set PWM for LED Driver"); }
}


void PCA9685::setPin(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t num, uint16_t val, bool invert) {    
    if(val > (uint16_t)4095) { val = (uint16_t)4095; }
    if (invert) { setPWM(client, srv, num, 4096, 0); }
    else if (val == 4095) { setPWM(client, srv, num, 0, 4096); }
    else {
        if (val == 4096) { setPWM(client, srv, num, 4096, 0); }
        else if (val == 0) { setPWM(client, srv, num, 0, 4096); }
        else { setPWM(client, srv, num, 0, val); }
    }
}


uint8_t PCA9685::read8(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t addr) {
    srv.request.request.resize(3);
    srv.request.size = 3;
    srv.request.bus = 1;
    srv.request.request = {0x01, _i2caddr, addr};

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while(!status_req(client, srv, _i2caddr));
        return (uint8_t)srv.response.data.at(0);
    } 
    else { ROS_ERROR("Unable to read from LED Driver"); }
}


void PCA9685::write8(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t addr, uint8_t d) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.bus = 1;
    srv.request.request = {0x02, _i2caddr, addr, d};

    if (client.call(srv)) {
        /* wait for job to be served in the i2c manager. */
        while (!status_req(client, srv, _i2caddr));
    } 
    else { ROS_ERROR("Unable to write for LED Driver"); }
}


void pca9685_init() {
    ros::ServiceClient client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;

    unsigned char addr = 0x40;
	PCA9685 pwm;

	pwm = PCA9685(addr);
    pwm.begin(client, srv);
	pwm.setPWMFreq(client, srv, 1600);
}

bool handle_req(hbs2::led::Request &req, hbs2::led::Response &res) {
    ros::ServiceClient client = n->serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;

    unsigned char addr = 0x40;
	PCA9685 pwm;
	pwm = PCA9685(addr);

    switch(req.color) {    
        // red
        case 1: {
            pwm.setPWM(client, srv, 0, 0, 2048);
            break;
        }
        // blue
        case 2: {
            pwm.setPWM(client, srv, 2, 0, 2048);
            break;
        }
        // green
        case 3: {
            pwm.setPWM(client, srv, 1, 0, 2048);
            break;
        }
        // yellow
        case 4: {
            pwm.setPWM(client, srv, 0, 0, 2048);
            pwm.setPWM(client, srv, 1, 0, 1024);
            break;
        }
        // white
        case 5: {
            pwm.setPWM(client, srv, 0, 0, 2048);
            pwm.setPWM(client, srv, 2, 0, 2048);
            pwm.setPWM(client, srv, 1, 0, 2048);
            break;
        }
        // pink
        case 6: {
            pwm.setPWM(client, srv, 0, 0, 2048);
            pwm.setPWM(client, srv, 1, 0, 512);
            break;
        }
        // off
        case 0: {
            pwm.setPWM(client, srv, 2, 0, 0);
            pwm.setPWM(client, srv, 1, 0, 0);
            pwm.setPWM(client, srv, 0, 0, 0);
            break;
        }
        default:
            break;
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pca9685");
    n = ros::NodeHandlePtr(new ros::NodeHandle);

    pca9685_init();
    ros::ServiceServer srv = n->advertiseService("led_srv", handle_req);
    ROS_INFO("ROS LED service has started.");

    ros::spin();

	return 0;
}
