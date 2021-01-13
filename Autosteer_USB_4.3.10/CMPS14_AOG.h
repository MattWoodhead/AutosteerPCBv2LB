/*
CMPS14_AOG.h
written for https://github.com/MattWoodhead/AutosteerPCBv2LB
copyright 2020 Matt Woodhead
GNU GPL v3.0
*/

#ifndef CMPS14_AOG_H_
#define CMPS14_AOG_H_

#include <Arduino.h>
#include <Wire.h>

// I2C address set in hardware (tied high or low)

#define CMPS14_ADDRESS 0x60  // Address of CMPS14 shifted right one bit for arduino wire library

//Register Map
#define CONTROL_REG 0           // Command register (write), Software version (read)
#define HEADING_8BIT 1          // Register to read 8bit yaw angle from
#define HEADING_MSB 2           // Register to read MSB of 16bit yaw angle from
#define HEADING_LSB 3           // Register to read LSB of 16bit yaw angle from
#define PITCH_8BIT 4                 // Register to read Pitch angle from
#define ROLL_8BIT 5             // Register to read 8bit Roll angle from
#define RAW_MAG_X_MSB 6         // Register to read MSB of 16bit raw magnetometer x value
#define RAW_MAG_X_LSB 7         // Register to read LSB of 16bit raw magnetometer x value
#define RAW_MAG_Y_MSB 8         // Register to read MSB of 16bit raw magnetometer y value
#define RAW_MAG_Y_LSB 9         // Register to read LSB of 16bit raw magnetometer y value
#define RAW_MAG_Z_MSB 10        // Register to read MSB of 16bit raw magnetometer z value
#define RAW_MAG_Z_LSB 11        // Register to read LSB of 16bit raw magnetometer z value
#define RAW_ACC_X_MSB 12        // Register to read MSB of 16bit raw accelerometer x value
#define RAW_ACC_X_LSB 13        // Register to read LSB of 16bit raw accelerometer x value
#define RAW_ACC_Y_MSB 14        // Register to read MSB of 16bit raw accelerometer y value
#define RAW_ACC_Y_LSB 15        // Register to read LSB of 16bit raw accelerometer y value
#define RAW_ACC_Z_MSB 16        // Register to read MSB of 16bit raw accelerometer z value
#define RAW_ACC_Z_LSB 17        // Register to read LSB of 16bit raw accelerometer z value
#define RAW_GYR_X_MSB 18        // Register to read MSB of 16bit raw gyroscope x value
#define RAW_GYR_X_LSB 19        // Register to read LSB of 16bit raw gyroscope x value
#define RAW_GYR_Y_MSB 20        // Register to read MSB of 16bit raw gyroscope y value
#define RAW_GYR_Y_LSB 21        // Register to read LSB of 16bit raw gyroscope y value
#define RAW_GYR_Z_MSB 22        // Register to read MSB of 16bit raw gyroscope z value
#define RAW_GYR_Z_LSB 23        // Register to read LSB of 16bit raw gyroscope z value
// Unknown 24-27
#define ROLL_MSB 28             // Register to read MSB of 16bit roll angle from
#define ROLL_LSB 29             // Register to read LSB of 16bit roll angle from
#define CALIBRATION 30          // Register to read LSB of 16bit roll angle from


class CMPS14
{
  public:
    CMPS14(uint8_t i2cAddress = CMPS14_ADDRESS);
      bool init();
      float getHeading();
      float getRoll();
      uint8_t getPitch();
      uint8_t softwareVersion;

      uint8_t CMPS_ADDRESS;
  private:
      uint8_t HeadingMSB, HeadingLSB, RollMSB, RollLSB;
      uint8_t Pitch;
      uint8_t readValue;
      float Heading, Roll;
};

#endif // CMPS14_AOG_H_
