/*  ADS1115 ADC hardware node */

// System
#include <unistd.h>
#include <stdlib.h>

// Custom
#include "ADS1115.h"

// ROS
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"


bool status_req(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t address) {
    srv.request.request.resize(4);
    srv.request.size = 4;
    srv.request.request = {0x00, address, 0x00, 0x00};
    usleep(1000);
    client.call(srv);
    if (!srv.response.success) return false;
    else return true;
}


//    @brief  Writes 16-bits to the specified destination register
static void writeRegister(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  srv.request.request.resize(5);
  srv.request.size = 5;
  srv.request.request = {0x02, i2cAddress, reg, (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)};

  if (client.call(srv)) {
    while(!status_req(client, srv, i2cAddress));
  } 
  else { 
    ROS_ERROR("Write request to ADC failed."); // exit(1);
  }
}


//    @brief  Reads 16-bits from specified register
static uint16_t readRegister(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t i2cAddress, uint8_t reg) {
  uint16_t value = 0;
  srv.request.request.resize(4);
  srv.request.size = 4;
  srv.request.request = {0x01, i2cAddress, reg, 0x00};

  if (client.call(srv)) {
    while(!status_req(client, srv, i2cAddress));
    value = (uint16_t)srv.response.data.at(0) << 8;
    value |= (uint16_t)srv.response.data.at(1);
    return value;
  } 
  else { 
    ROS_ERROR("Read request to the ADC failed."); // exit(1);
  } 
}


//    @brief  Instantiates a new ADS1115 class w/appropriate properties
ADS1115::ADS1115(uint8_t i2cAddress)
{
   m_i2cAddress = i2cAddress;
   m_conversionDelay = ADS1115_CONVERSIONDELAY;
   m_bitShift = 0;
   m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
}


//    @brief  Sets up the HW (reads coefficients values, etc.)
void ADS1115::begin() {
  
}


//    @brief  Sets the gain and input voltage range
void ADS1115::setGain(adsGain_t gain)
{
  m_gain = gain;
}


//    @brief  s a gain and input voltage range
adsGain_t ADS1115::getGain()
{
  return m_gain;
}


//    @brief  Gets a single-ended ADC reading from the specified channel
uint16_t ADS1115::readADC_SingleEnded(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t channel) {
  if (channel > 3)
  {
    return 0;
  }
  
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(8000);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return readRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
}

 
/*    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative. */
int16_t ADS1115::readADC_Differential_0_1(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;
                    
  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(8000);

  // Read the conversion results
  uint16_t res = readRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

 
/*    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative. */
int16_t ADS1115::readADC_Differential_2_3(ros::ServiceClient &client, hbs2::i2c_bus &srv) {
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(8000);

  // Read the conversion results
  uint16_t res = readRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}


/*    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.
            This will also set the ADC in continuous conversion mode.   */
void ADS1115::startComparator_SingleEnded(ros::ServiceClient &client, hbs2::i2c_bus &srv, uint8_t channel, int16_t threshold)
{
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;
                    
  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}


/*    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value. */
int16_t ADS1115::getLastConversionResults(ros::ServiceClient &client, hbs2::i2c_bus &srv)
{
  // Wait for the conversion to complete
  usleep(8000);

  // Read the conversion results
  uint16_t res = readRegister(client, srv, m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

// The ADC input range (or gain) can be changed via the following
// functions, but be careful never to exceed VDD +0.3V max, or to
// exceed the upper and lower limits if you adjust the input range!
// Setting these values incorrectly may destroy your ADC!
//                                                                ADS1015  ADS1115
//                                                                -------  -------
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ads1115");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hbs2::i2c_bus>("i2c_srv");
  hbs2::i2c_bus srv;

  ADS1115 ads;
  
  int16_t adc0, adc1, adc2, adc3;

  // Create publisher:
  ros::Publisher adc_pub = n.advertise<std_msgs::Int16MultiArray>("tpc_adc", 5);
  // Running at 10Hz:
  ros::Rate loop_rate(2);

  while(ros::ok) {        
      adc0 = ads.readADC_SingleEnded(client, srv, 0);
      adc1 = ads.readADC_SingleEnded(client, srv, 1);
      adc2 = ads.readADC_SingleEnded(client, srv, 2);
      adc3 = ads.readADC_SingleEnded(client, srv, 3);

      // Store data in message object and then publish
      std_msgs::Int16MultiArray msg;
      // Clear array
      msg.data.clear();
      msg.data.push_back(adc0);
      msg.data.push_back(adc1);
      msg.data.push_back(adc2);
      msg.data.push_back(adc3);

      // Broadcast message to anyone connected:
      adc_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}