#ifndef  PCA9685_H
#define  PCA9685_H

// System
#include <stdint.h>

// ROS
#include "ros/ros.h"
#include "hbs2/i2c_bus.h"

#define PCA9685_SUBADR1   0x02
#define PCA9685_SUBADR2   0x03
#define PCA9685_SUBADR3   0x04

#define PCA9685_MODE1     0x00
#define PCA9685_PRESCALE  0xFE

#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09

#define ALLLED_ON_L       0xFA
#define ALLLED_ON_H       0xFB
#define ALLLED_OFF_L      0xFC
#define ALLLED_OFF_H      0xFD

//#define ENABLE_DEBUG_OUTPUT 1

class PCA9685 
{
    public:

    // enum Color {
    //     RED,
    //     GREEN,
    //     BLUE,
    //     PINK,
    //     WHITE,
    //     YELLOW
    // };

     PCA9685(uint8_t addr = 0x40);
        void begin(ros::ServiceClient &client, hbs2::i2c_bus &srv);
        void reset(ros::ServiceClient &client, hbs2::i2c_bus &srv);
        void setPWMFreq(ros::ServiceClient &client, hbs2::i2c_bus &srv, float freq);
        void setPWM(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t, uint16_t, uint16_t);
        void setPin(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t num, uint16_t val, bool invert=false);       

        uint8_t _RedPin;
        uint8_t _GreenPin;
        uint8_t _BluePin;
    
    private:
        uint8_t _i2caddr;
        uint8_t null = 0;

        void    write8(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t, uint8_t d);
        uint8_t read8(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t addr);
};

#endif
