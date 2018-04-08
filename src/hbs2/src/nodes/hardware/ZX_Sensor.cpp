/*  Gesture sensor hardware node    */

// System
#include <unistd.h>
#include <stdlib.h>

// Custom
#include "ZX_Sensor.h"

// ROS
#include "std_msgs/UInt8.h"

/**
 * Constructor - Instantiates ZX_Sensor object
 */
ZX_Sensor::ZX_Sensor(int address)
{
    addr_ = address;
}

/**
 * Destructor
 */
ZX_Sensor::~ZX_Sensor()
{

}

/**
 * Configures I2C communications and checks ZX sensor model number.
 *
 * @param enable_interrupts enables DR pin to assert on events
 * @return True if initialized successfully. False otherwise.
 */
bool ZX_Sensor::init(ros::ServiceClient &client, hbs2::i2c_bus &srv, InterruptType interrupts /* = NO_INTERRUPTS */, bool active_high /* = true */) {
    
    /* Initialize I2C */
    //Wire.begin();
    
    /* Enable DR interrupts based on desired interrupts */
    setInterruptTrigger(client, srv, interrupts);
    configureInterrupts(client, srv, active_high, false);
    if ( interrupts == NO_INTERRUPTS ) {
        disableInterrupts(client, srv);
    } else {
        enableInterrupts(client, srv);
    }
    
    return true;
}

/**
 * Reads the sensor model version
 *
 * @return sensor model version number
 */
uint8_t ZX_Sensor::getModelVersion(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t ver;
    
    if ( !wireReadDataByte(client, srv, ZX_MODEL, ver) ) {
        return ZX_ERROR;
    }

    return ver;
}

/**
 * Reads the register map version
 *
 * @return register map version number
 */
uint8_t ZX_Sensor::getRegMapVersion(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t ver;
    
    if ( !wireReadDataByte(client, srv, ZX_REGVER, ver) ) {
        return ZX_ERROR;
    }
    
    return ver;
}

/*******************************************************************************
 * Interrupt Configuration
 ******************************************************************************/

 /**
  * Sets the triggers that cause the DR pin to change
  *
  * @param[in] interrupts which types of interrupts to enable
  * @return True if operation successful. False otherwise.
  */
bool ZX_Sensor::setInterruptTrigger(ros::ServiceClient &client, hbs2::i2c_bus &srv, InterruptType interrupts) {
    
#if DEBUG
    Serial.printf("Setting interrupts: "));
    Serial.println(interrupts);
#endif

    switch ( interrupts ) {
        case POSITION_INTERRUPTS:
            if ( !setRegisterBit(client, srv, ZX_DRE, DRE_CRD) ) {
                return false;
            }
            break;
        case GESTURE_INTERRUPTS:
            if ( !setRegisterBit(client, srv, ZX_DRE, DRE_SWP) ) {
                return false;
            }
            if ( !setRegisterBit(client, srv, ZX_DRE, DRE_HOVER) ) {
                return false;
            }
            if ( !setRegisterBit(client, srv, ZX_DRE, DRE_HVG) ) {
                return false;
            }
            break;
        case ALL_INTERRUPTS:
            if ( !wireWriteDataByte(client, srv, ZX_DRE, SET_ALL_DRE) ) {
                return false;
            }
            break;
        default:
            if ( !wireWriteDataByte(client, srv, ZX_DRE, 0x00) ) {
                return false;
            }
            break;
    }
    
#if DEBUG
    uint8_t val;
    wireReadDataByte(client, srv, ZX_DRE, val);
    Serial.print(F("ZX_DRE: b"));
    Serial.println(val, BIN);
#endif
    
    return true;
}

/**
 * Configured the behavior of the DR pin on an interrupt
 *
 * @param[in] active_high true for DR active-high. False for active-low.
 * @param[in] pin_pulse true: DR pulse. False: DR pin asserts until STATUS read
 * @return True if operation successful. False otherwise.
 */
bool ZX_Sensor::configureInterrupts(ros::ServiceClient &client, hbs2::i2c_bus &srv, bool active_high, bool pin_pulse /* = false */)
{
    /* Set or clear polarity bit to make DR active-high or active-low */
    if ( active_high ) {
        if ( !setRegisterBit(client, srv, ZX_DRCFG, DRCFG_POLARITY) ) {
            return false;
        }
    } else {
        if ( !clearRegisterBit(client, srv, ZX_DRCFG, DRCFG_POLARITY) ) {
            return false;
        }
    }
    
    /* Set or clear edge bit to make DR pulse or remain set until STATUS read */
    if ( pin_pulse ) {
        if ( !setRegisterBit(client, srv, ZX_DRCFG, DRCFG_EDGE) ) {
            return false;
        }
    } else {
        if ( !clearRegisterBit(client, srv, ZX_DRCFG, DRCFG_EDGE) ) {
            return false;
        }
    }
    
#if DEBUG
    uint8_t val;
    wireReadDataByte(client, srv, ZX_DRCFG, val);
    Serial.print(F("ZX_DRCFG: b"));
    Serial.println(val, BIN);
#endif
    
    return true;
}

/**
 * Turns on interrupts so that DR asserts on desired events.
 *
 * @return True if operation successful. False otherwise.
 */
bool ZX_Sensor::enableInterrupts(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    if ( !setRegisterBit(client, srv, ZX_DRCFG, DRCFG_EN) ) {
        return false;
    }
    
    return true;
}

/**
 * Turns off interrupts so that DR will never assert.
 *
 * @return True if operation successful. False otherwise.
 */
bool ZX_Sensor::disableInterrupts(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    if ( !clearRegisterBit(client, srv, ZX_DRCFG, DRCFG_EN) ) {
        return false;
    }
    
    return true;
}

/**
 * Reads the STATUS register to clear an interrupt (de-assert DR)
 *
 * @return True if operation successful. False otherwise.
 */
bool ZX_Sensor::clearInterrupt(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t val;
    
    if ( !wireReadDataByte(client, srv, ZX_STATUS, val) ) {
        return false;
    }
    
    return true;
}

/*******************************************************************************
 * Data available
 ******************************************************************************/

 /**
  * Indicates that new position (X or Z) data is available
  *
  * @return True if data is ready to be read. False otherwise.
  */
bool ZX_Sensor::positionAvailable(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t status;
    
    /* Read STATUS register and extract DAV bit */
    if ( !wireReadDataByte(client, srv, ZX_STATUS, status) ) {
        return false;
    }
    status &= 0b00000001;    
    if ( status ) {
        return true;
    }
    
    return false;
}

/**
 * Indicates that a gesture is available to be read
 *
 * @return True if gesture is ready to be read. False otherwise.
 */
bool ZX_Sensor::gestureAvailable(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t status;
    
    /* Read STATUS register and extract SWP bit */
    if ( !wireReadDataByte(client, srv, ZX_STATUS, status) ) {
        return false;
    }
    
    status &= 0b00011100;    
    if ( status ) {
        return true;
    }
    
    return false;
}

/*******************************************************************************
 * Sensor data read
 ******************************************************************************/

/**
 * Reads the X position data from the sensor
 *
 * @return 0-240 for X position. 0xFF on read error.
 */
uint8_t ZX_Sensor::readX(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t x_pos;
    
    /* Read X Position register and return it */
    if ( !wireReadDataByte(client, srv, ZX_XPOS, x_pos) ) {
        return ZX_ERROR;
    }
    if ( x_pos > MAX_X ) {
        return ZX_ERROR;
    }
    return x_pos;
}

/**
 * Reads the Z position data from the sensor
 *
 * @return 0-240 for Z position. 0xFF on read error.
 */
uint8_t ZX_Sensor::readZ(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t z_pos;
    
    /* Read X Position register and return it */
    if ( !wireReadDataByte(client, srv, ZX_ZPOS, z_pos) ) {
        return ZX_ERROR;
    }
    if ( z_pos > MAX_Z ) {
        return ZX_ERROR;
    }
    return z_pos;
}

/**
 * Reads the last detected gesture from the sensor
 *
 * 0x01 Right Swipe
 * 0x02 Left Swipe
 * 0x03 Up Swipe
 *
 * @return a number corresponding to  a gesture. 0xFF on error.
 */
GestureType ZX_Sensor::readGesture(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t gesture;
    
    /* Read GESTURE register and return the value */
    if ( !wireReadDataByte(client, srv, ZX_GESTURE, gesture) ) {
        printf("UNABLE TO READ GESTURE REG\n");
        return NO_GESTURE;
    }
#if DEBUG
    Serial.print(F("Gesture read: "));
    Serial.println(gesture);
#endif
    printf("Gesture: %u\n", gesture);
    switch ( gesture ) {
        case RIGHT_SWIPE:
            return RIGHT_SWIPE;
        case LEFT_SWIPE:
            return LEFT_SWIPE;
        case UP_SWIPE:
            return UP_SWIPE;
        default:
            return NO_GESTURE;
    }
}

/**
 * Reads the speed of the last gesture from the sensor
 *
 * @return a number corresponding to the speed of the gesture. 0xFF on error.
 */
uint8_t ZX_Sensor::readGestureSpeed(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
    uint8_t speed;
    
    /* Read GESTURE register and return the value */
    if ( !wireReadDataByte(client, srv, ZX_GSPEED, speed) ) {
        return ZX_ERROR;
    }
    
    return speed;
}

/*******************************************************************************
 * Bit Manipulation
 ******************************************************************************/
 
/**
 * sets a bit in a register over I2C
 *
 * @param[in] bit the number of the bit (0-7) to set
 * @return True if successful write operation. False otherwise.
 */
bool ZX_Sensor::setRegisterBit(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t reg, uint8_t bit) {
    uint8_t val;
    
    /* Read value from register */
    if ( !wireReadDataByte(client, srv, reg, val) ) {
        return false;
    }
    
    /* Set bits in register and write back to the register */
    val |= (1 << bit);
    if ( !wireWriteDataByte(client, srv, reg, val) ) {
        return false;
    }
    
    return true;    
}

/**
 * clears a bit in a register over I2C
 *
 * @param[in] bit the number of the bit (0-7) to clear
 * @return True if successful write operation. False otherwise.
 */
bool ZX_Sensor::clearRegisterBit(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t reg, uint8_t bit) {
    uint8_t val;
    
    /* Read value from register */
    if ( !wireReadDataByte(client, srv, reg, val) ) {
        return false;
    }
    
    /* Clear bit in register and write back to the register */
    val &= ~(1 << bit);
    if ( !wireWriteDataByte(client, srv, reg, val) ) {
        return false;
    }
    
    return true;    
} 

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t address) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.request = {0x00, address, 0x00, 0x00};
    usleep(1000);
    client.call(srv);
    if (!srv.response.success) return false;
    else return true;
}

/**
 * Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool ZX_Sensor::wireWriteDataByte(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t reg, uint8_t val) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.request = {0x02, addr_, reg, val};

    if (client.call(srv)) {
        while (!status_req(client, srv, addr_));
        return true;
    } else { ROS_ERROR("Unable to write for gesture sensor"); return false; }    
}

/**
 * Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool ZX_Sensor::wireReadDataByte(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t reg, uint8_t &val) {
    srv.request.request.resize(3);
    srv.request.size = 3;
    srv.request.request = {0x01, addr_, reg};

    if (client.call(srv)) {
        while(!status_req(client, srv, addr_));
        val = (uint8_t)srv.response.data.at(0);
        return true;
    } else { ROS_ERROR("Unable to read from gesture sensor"); return false; }    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zx_sensor");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hbs2::i2c_bus>("i2c_srv");
    hbs2::i2c_bus srv;

    // Create publisher:
    ros::Publisher gesture_pub = n.advertise<std_msgs::UInt8>("tpc_wave", 5);

    // Running at 10Hz:
    ros::Rate loop_rate(10);    

    ZX_Sensor zx_sensor = ZX_Sensor(0x10);
    uint8_t x_pos[3];
    uint8_t z_pos[3];

    // Initialize ZX Sensor (configure I2C and read model ID)
    if ( zx_sensor.init(client, srv) ) {
        printf("ZX Sensor initialization complete");
    } else {
        printf("Something went wrong during ZX Sensor init!");
    }

	while(ros::ok()) {
        // Message to be published to wave topic:
        std_msgs::UInt8 msg;

		// Store 10 position readings
		for (int i = 0; i < 3; ) {
			if ( zx_sensor.positionAvailable(client, srv) ) {
				x_pos[i] = zx_sensor.readX(client, srv);
				z_pos[i] = zx_sensor.readZ(client, srv);
				i++;
			}
		}

		int i = 0;
		for (i = 0; i < 3; i++) {
			if (z_pos[i] > 20)
				break;
		}
		if (i == 3) {
			ROS_INFO("Wave");
            msg.data = 1;
			sleep(1);			
		} else { msg.data = 0; }

        // Broadcast message to subscriber
        gesture_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
	}
	return 1;
}
