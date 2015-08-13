/*
  MPL3115A2.cpp
  This .cpp and the corresponding .h file provide a library of procedures for
  collecting readings from the Freescale MPL3115A2 altitude/barometric pressure/temperature
  sensor.

  This library uses Truchsess's I2C library for communication with the MPL3115A2 to avoid
  the misaligned data caused by repeated-starts with the older Wire library.

  Sampling found to be unreliable over time when using the Wire library, <wire.h>
  Converted to use I2C library since it handles the repeated START I2C commands that the 
  Freescale, Inc., devices use.

  Thanks to Wayne Truchess for his excellent work on that system.
  See http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library

  Written by HDTodd, July, 2015
  Based on code by: A.Weiss, 7/17/2012, changes Nathan Seidle Sept 23rd, 2013 (SparkFun)
  License: This code is public domain but you buy me (Seidle) a beer if you use this 
   and we meet someday (Beerware license).
 
  Hardware Connections to MPL3115A2 (Breakoutboard to Arduino):
    -VCC = 3.3V
    -SDA = A4		Add 330 ohm resistor in series if using 5v VCC
    -SCL = A5		Add 330 ohm resistor in series if using 5v VCC
    -INT pins can be left unconnected for this demo
 
    Usage:
      -Serial terminal at 9600bps
      -Times various measurements
      -Examines status flags uses to poll device for data ready
 
  During testing, GPS with 9 satellites reported 5393ft, sensor reported 5360ft 
    (delta of 33ft). Very close! (Seidle)
  During testing, GPS with 9 satellites reported 1520m; sensor reported 1433m; 
    surprisingly far off (Todd)
    [So I chose to tell it what the altitude is and have it apply the correction -- Todd]
 */

#include <I2C.h> // for IIC communication
#include "MPL3115A2.h"

// Instantiate the class
MPL3115A2::MPL3115A2() {
}

boolean MPL3115A2::begin() {
  uint8_t eC, whoCode;

//  Assumes I2c.begin and Serial.begin have already been invoked
//    and that the I2C speed has been set as desired.  

// Confirm that we can find the device
  eC=I2c.read(MPL3115A2_ADDR, WHO_AM_I, 1, &whoCode);
  if ( eC != 0 ) {      // fatal error condition
       Serial.println("[MPL3115A2]No response from MPL3115A2 - check connections");
       Serial.print("I2C error code = 0x"); Serial.println(eC, HEX);
       Serial.println("Exiting ..."); 
       delay(1000);     // wait for tty output to complete       
       exit(1);
  };

// Confirm that we're talking to the right thing
  if ( whoCode != I_AM_MPL3115A2 ) {
    return false;
  };

// Configure the sensor
// First, reset MPL3115A2 to known default state
  reset();

// Now set our operating characteristics  
  setModeStandby();        // Ensure standby mode before writing CTRL REG
                           // Trigger samplings on demand 
  enableEventFlags();     // Enable all three pressure and temp event flags

  return true;
}

// Sets altitude correction.  Not generally used, but available if needed
// myAltitude is your GPS-supplied altitude, in meters
// Note that correction is only -128..+127 m, limited by 8-bit field size in device
void MPL3115A2::setAltitude(int myAltitude) {
  if (myAltitude!=0) 
    writeCtl(OFF_H,(int8_t)(myAltitude-(int)readAltitude()), (char *) "altitudeCorrection");
};


//Returns the number of meters above sea level
float MPL3115A2::readAltitude() {
  
  uint32_t alt;
  uint8_t sReg, dReg,  msb, csb, lsb;
  int counter;
  long start;

//  Make sure there's nothing in the device buffer
  readStatus(STATUS, &sReg,(char *) "readAltitude() buffer-clear");
  if ( ( sReg & (PDR_F) ) != 0) 
    readStatus(OUT_P_MSB, &dReg, (char *) "readAltitude() buffer-clear");  // read data to clear flag
  readStatus(STATUS, &sReg,(char *) "readAltitude() buffer-clear");       // double-check it
  if ( ( sReg & (PDR_F) ) != 0) Serial.println("Pressure data ready at start of altitude!");
  setModeAltimeter();     // Measure altitude above sea level in meters
  toggleOneShot();        // Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for PDR bit, indicates we have new pressure data
  counter = 0;
  start = millis(); 
  I2c.read(MPL3115A2_ADDR, STATUS, 1, &sReg);
  while ( ( sReg & (PDR_F) ) == 0 )
  {
      if ( ( ++counter > 1000) | (millis() > start+TIMEOUT) ) {
        Serial.println("Timeout waiting for altitude reading");
        return(-999); //Error out
      }
      delay(1);
      I2c.read(MPL3115A2_ADDR, STATUS, 1, &sReg);
  }
  //  Serial.print("  Alt read time "); Serial.print(millis()-start); Serial.print("ms  ");

  readStatus(OUT_P_MSB, &msb, (char *) "readAltitude() getting msb");
  readStatus(OUT_P_CSB, &csb, (char *) "readAltitude() getting csb");
  readStatus(OUT_P_LSB, &lsb, (char *) "readAltitude() getting lsb");

  // The least significant byte of l_altitude is a left-justified 4-bit
  // fractional value in Pascals.

  alt = (uint32_t)msb<<16 | (uint32_t)csb<<8 | (uint32_t)(lsb & 0xF0);
  return ( (float)alt/256.0 );
}

//Returns the number of feet above sea level
float MPL3115A2::readAltitudeFt()
{
  return ( readAltitude() * FT_PER_METER );
}

//Reads the current pressure in Pa
//Unit must be set in barometric pressure mode
float MPL3115A2::readPressure()
{
  uint32_t  press;
  uint8_t sReg, dReg,  msb, csb, lsb;
  int counter;
  long start;

//  Make sure there's nothing in the device buffer
  readStatus(STATUS, &sReg, (char *) "readPressure() buffer-clear");
  if ( ( sReg & (PDR_F) ) != 0) 
    readStatus(OUT_P_MSB, &dReg, (char *) "readAltitude() buffer-clear");  // read MSB to clear flag
  readStatus(STATUS, &sReg,(char *) "readPressure() buffer-clear");       // double-check it
  if ( ( sReg & (PDR_F) ) != 0) Serial.println("Pressure data ready at start of altitude!");
  setModeBarometer();     // Measure pressure in Pascals from 20 to 110 kPa
  toggleOneShot();        //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for PDR bit, indicates we have new pressure data
  counter = 0;
  start = millis(); 
  readStatus(STATUS, &sReg, (char *) "pressure() data sampling");
  I2c.read(MPL3115A2_ADDR, STATUS, 1, &sReg);
  while ( ( sReg & (PDR_F)) == 0 )
  {
      if ( ( ++counter > 1000) | (millis() > start+TIMEOUT) ) {
        Serial.println("Timeout waiting for pressure");
        return(-999); //Error out
      }
      delay(1);
      readStatus(STATUS, &sReg, (char *) "pressure() data sampling");
  }

  //  Serial.print("Pressure read time "); Serial.print(millis()-start); Serial.print("ms  ");

  readStatus(OUT_P_MSB, &msb, (char *) "readPressure() getting msb");
  readStatus(OUT_P_CSB, &csb, (char *) "readPressure() getting csb");
  readStatus(OUT_P_LSB, &lsb, (char *) "readPressure() getting lsb");

  press = (uint32_t)msb<<16 | (uint32_t)csb<<8 | (uint32_t)(lsb & 0xF0);
  return( (float)press/64.0 );
}

//  returns the temperature in Celsius
float MPL3115A2::readTemp() {
  uint8_t sReg,dReg, msb, lsb;
  int counter;
  long start;


//  Make sure there's nothing in the device buffer
  readStatus(STATUS, &sReg, (char *) "readTemp() buffer-clear");
  if ( ( sReg & (TDR_F) ) != 0) 
    readStatus(OUT_T_MSB, &dReg, (char *) "readTemp() buffer-clear");  // read MSB to clear flag
  readStatus(STATUS, &sReg, (char *) "readTemp() 2nd buffer-clear");       // double-check it
  if ( ( sReg & (TDR_F) ) != 0) Serial.println("Temp data ready at start of readTemp()!");

  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for TDR bit, indicates we have new temp data
  counter = 0;
  start = millis(); 
  readStatus(STATUS, &sReg, (char *) "readTemp() data sampling");
  while( ( sReg & (PDR_F)) == 0 ) {
      if ( ( ++counter > 1000) | (millis() > start+TIMEOUT) ) {
        Serial.println("Timeout waiting for temperature");
        return(-999); //Error out
      }
      delay(1);
      readStatus(STATUS, &sReg, (char *) "readTemp() data sampling"); 
      }
  //  Serial.print("  Temp read time "); Serial.print(millis()-start); Serial.print("ms  ");
  
  // Read temperature registers

  readStatus(OUT_T_MSB, &msb, (char *) "readPressure() getting msb");
  readStatus(OUT_T_LSB, &lsb, (char *) "readPressure() getting lsb");

/*  Temp is 8-bit integer +  4-bit fraction, 2s-comp format, left-justified 
 *  in two successive bytes.  Create 16-bit unsigned, convert to signed, 
 *  & divide by 256 in floating to get to 2s-comp float with decimal pt
 *  located between the two bytes.
*/
  return ( (float) ( (int16_t) ((uint16_t)msb<<8 | ( (uint16_t)lsb & 0xF0) ) )/256.0 );
}

//Give me temperature in fahrenheit!
float MPL3115A2::readTempF()
{
  return((readTemp() * 9.0)/ 5.0 + 32.0); // Convert celsius to fahrenheit
}

//Sets the mode to Barometer
//CTRL_REG1, ALT bit
void MPL3115A2::setModeBarometer()
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setModeBarometer()"); 
  cReg &= ~(ALT_F);                             // Clear ALT bit
  writeCtl(CTRL_REG1, cReg, (char *) "setModeBarometer()");
}

//Sets the mode to Altimeter
//CTRL_REG1, ALT bit
void MPL3115A2::setModeAltimeter()
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setModeAltimeter()"); 
  cReg |= (ALT_F);                              // Set ALT bit
  writeCtl(CTRL_REG1, cReg, (char *) "setModeAltimeter()");
}

//Puts the sensor in standby mode
//This is needed so that we can modify the major control registers
void MPL3115A2::setModeStandby()
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setModeStandby()"); 
  cReg &= ~(ACTV_F);                          // Clear ACTIVE bit
  writeCtl(CTRL_REG1, cReg, (char *) "setModeStandby()");
}

//Puts the sensor in active mode
//This is needed so that we can modify the major control registers
void MPL3115A2::setModeActive()
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setModeActive()"); 
  cReg |= (ACTV_F);                          // Set ACTIVE bit
  writeCtl(CTRL_REG1, cReg, (char *) "setModeActive()");
}

//Set up FIFO mode to one of three modes. See page 26, table 31 of MPL3115A datasheet
//From user jr4284
void MPL3115A2::setFIFOMode(uint8_t f_Mode)
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setMoFIFOMode()"); 
  if (f_Mode > 3) f_Mode = 3;                   // FIFO value cannot exceed 3.
  f_Mode <<= 6;                                 // Shift FIFO byte left 6 to put it in bits 6, 7.
  cReg &= ~(3<<6);                              // clear bits 6, 7
  cReg |= f_Mode;                               // Mask in new FIFO bits
  writeCtl(CTRL_REG1, cReg, (char *) "setMoFIFOMode()");
}

//Call with a rate from 0 to 7. See page 33 for table of ratios.
//Sets the over sample rate. Datasheet calls for 128 but you can set it 
//from 1 to 128 samples. The higher the oversample rate the greater
//the time between data samples.
void MPL3115A2::setOversampleRate(uint8_t sampleRate)
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setOversampleRate()"); 
  if(sampleRate > 7) sampleRate = 7;            // OS cannot be larger than 0b.0111
  sampleRate <<= 3;                             // Align it for the CTRL_REG1 register bits 3,4,5
  cReg &= ~(7<<3);                              // Clear out old OS bits in bits 3,4,5
  cReg |= sampleRate;                           // Mask in new OS bits
  writeCtl(CTRL_REG1, cReg, (char *) "setOversampleRate()");    // Write 8t to reg & done
}

// Returns the current oversample rate setting, a value 0-7.
// See page 33 for table of rate settings.
uint8_t MPL3115A2::getOversampleRate(void)
{
  uint8_t sampleRate;
  uint8_t eC, msb, csb, lsb, cReg, sReg, dReg;

  readStatus(CTRL_REG1, &cReg, (char *) "setOversampleRate()"); 
  sampleRate = (cReg >>= 3) & 0b111;
  return (sampleRate );  // right justify, isolate, & return that 3 bit field
}

//Clears then sets the OST bit which causes the sensor to immediately take another reading
//Needed to sample faster than 1Hz
void MPL3115A2::toggleOneShot(void)
{
  uint8_t cReg;

  readStatus(CTRL_REG1, &cReg, (char *) "toggleOneShot()"); 
  cReg &= ~(OST_F);                             // Clear OST bit
  writeCtl(CTRL_REG1, cReg, (char *) "toggleOneShot()");    // Write it to reg & done

  readStatus(CTRL_REG1, &cReg, (char *) "toggleOneShot()"); //Read current settings to be safe
  cReg |= (OST_F);                              // Set OST bit
  writeCtl(CTRL_REG1, cReg, (char *) "toggleOneShot()");    // Write it to reg & done
}

// Reset MPL3115A2 to known default states and registers
void MPL3115A2::reset(void) {
  uint8_t resetCode = RST_F;
  long maxWait;
  int counter;
  uint8_t eC, cReg;

//  Write reset bit to have device reboot
//  No error checking here, since it will likely fail as device shuts off I2C during boot
  eC = I2c.write(MPL3115A2_ADDR, CTRL_REG1, RST_F);

//  Now wait for it to complete and for I2C communications to come back online
//  Generally seems to be only two iterations, 2ms or so
  maxWait = millis() + TIMEOUT;
  counter = 0;
  eC = I2c.read(MPL3115A2_ADDR, CTRL_REG1, &cReg);
  while ( eC!=0 || ( (cReg & RST_F)!=0) ) 
    {
      eC = I2c.read(MPL3115A2_ADDR,CTRL_REG1,1,&cReg);
      if ( ( ++counter > 1000) | (millis() > maxWait) ) {
        Serial.println("Timeout waiting for reset");
        Serial.print("I2C read error code = 0x"); Serial.println(eC, HEX);
        Serial.print("Ctrl Reg = 0x"); Serial.println(cReg, HEX);
        delay(1000);
        exit(3);       //Error out
      }
      delay(1);
    };
}

// Routine to write control reg, with error detection
// Parameters:
//    reg     register to be written
//    value   value to write
//    caller  char string identifying calling routine for error msgs
// Returns:
//    eC      0 if no error; otherwise Atmel I2C error code
uint8_t MPL3115A2::writeCtl(uint8_t reg, uint8_t value, char *caller) {
  uint8_t eC;

  eC = I2c.write((uint8_t) MPL3115A2_ADDR, (uint8_t) reg, (uint8_t) value);
  if (eC != 0) {
    Serial.print("I2C communication write error in "); Serial.print(caller); 
    Serial.print("; Atmel I2C error code = 0x"); Serial.println(eC, HEX);
  }
  return(eC);
}

// Routine to read control reg, with error detection
// Parameters:
//    reg     register to be written
//    value   address to store value read
//    caller  char string identifying calling routine for error msgs
// Returns:
//    eC      0 if no error; otherwise Atmel I2C error code
uint8_t MPL3115A2::readStatus(uint8_t reg, uint8_t *datain, char *caller) {
  uint8_t eC;

  eC = I2c.read(MPL3115A2_ADDR, reg, 1, datain);
  if ( eC != 0 ) {
    Serial.print("I2C communication read error in "); Serial.print(caller); 
    Serial.print("; Atmel I2C error code = 0x"); Serial.println(eC, HEX);    
  }
  return(eC);
}

//Enables the pressure and temp measurement event flags so that we can
//test against them. This is recommended in datasheet during setup.
void MPL3115A2::enableEventFlags()
{
  uint8_t eC;

  eC = I2c.write(MPL3115A2_ADDR, PT_DATA_CFG, 0x07);     // Enable all three pressure and temp event flags 
  if (eC!=0) { Serial.print("In enableEventFlags(), eC = 0x"); Serial.println(eC, HEX); }
}

