/*
CMPS14_AOG.cpp
written for https://github.com/MattWoodhead/AutosteerPCBv2LB
copyright 2020 Matt Woodhead
GNU GPL v3.0
*/

#include "CMPS14_AOG.h"

bool CMPS14::init()
{
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(HEADING_MSB);
  Wire.requestFrom(CMPS14_ADDRESS, 1);
  softwareVersion = Wire.read();
  if (softwareVersion != 255)
  {
	  return true;
  }
  else
  {
    return false;
  }
}

float CMPS14::getRoll()
{ 
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(ROLL_MSB);
  Wire.requestFrom(ROLL_MSB, 2);
  while(Wire.available() < 2);        // Wait for all bytes to come back
  RollMSB = Wire.read();
  RollLSB = Wire.read();
  Wire.endTransmission();
  Roll = (((int16_t)RollMSB << 8) | RollLSB)*0.1;
  return Roll;
}

float CMPS14::getHeading()
{
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(HEADING_MSB);
  Wire.requestFrom(CMPS14_ADDRESS, 2);
  while(Wire.available() < 2);        // Wait for all bytes to come back
  HeadingMSB = Wire.read();
  HeadingLSB = Wire.read();
  Wire.endTransmission();
  Heading = (((int16_t)HeadingMSB << 8) | HeadingLSB)*0.1;
  return Heading;
}

uint8_t CMPS14::getPitch()
{
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(PITCH_8BIT);
  Pitch = Wire.read();
  Wire.endTransmission();
  return Pitch;
}

/**************************************************************************/
// Instantiates a new CMPS14 class 
/**************************************************************************/
CMPS14::CMPS14(uint8_t i2cAddress) 
{
   CMPS_ADDRESS = i2cAddress;
}
