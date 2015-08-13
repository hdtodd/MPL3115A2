/*
  MPL3115A2.h
  This .h and the corresponding .cpp file provide a library of procedures for
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

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D

//  DR_STATUS flags
#define TDR_F   0x02    // temp data ready
#define PDR_F   0x04    // pressure/altitude data ready
#define PTDR_F  0x08    // P/A *or* Temp data ready
#define TOW_F   0x20    // Temp data was overwritten
#define POW_F   0x40    // Press/Altitude data was overwritten
#define PTOW_F  0x80    // P/A *or* temp data was overwritten

//  Control Register 1 flags
#define ACTV_F  0x01    // 0==>standby, 1==>active
#define OST_F   0x02    // -->1 triggers sampling
#define RST_F   0x04    // -->1 triggers reset; 0==> reset complete
#define OS_F    0x38    // 3-bit field for OverSample rate
#define RAW_F   0x40    // sample in raw mode
#define ALT_F   0x80    // 1==>altimeter mode; 0==>barometer mode

enum signalFlags {
        S_1=0,      // count 1, time 6ms
        S_2,        // count 2, time 10ms
        S_4,        // count 4, time 18ms
        S_8,        // count 8, time 34ms
        S_16,       // count 16, time 66ms
        S_32,       // count 32, time 130ms
        S_64,       // count 64, time 258ms
        S_128      // count 128, time 512ms
};

#define MPL3115A2_ADDR 0x60     // 7-bit I2C address
#define I_AM_MPL3115A2 0xC4     // returned from Reg 0x0C per datasheet spec

#define REPS 3       		// # of iterations for averaging timings
#define TIMEOUT 512             // max wait time in ms for device response
#define FT_PER_METER 3.28084    // conversion

class MPL3115A2 {
 public:
  MPL3115A2();
  boolean begin(void);
  void setAltitude(int myAltitude);  // sets altitude correction, in m, if needed
  float readAltitude(void);          // returns altitude in m
  float readAltitudeFt(void);         // returns altitude in ft
  float readPressure(void);          // returns pressure in Pascals
  float readTemp(void);              // returns temperature in Celsius
  float readTempF(void);             // returns temperature in Fahrenheit
  void setModeBarometer(void);
  void setModeAltimeter(void);
  void setModeStandby(void);
  void setModeActive(void);
  void setFIFOMode(uint8_t f_Mode);
  void setOversampleRate(uint8_t sampleRate);
  uint8_t getOversampleRate(void);
  void toggleOneShot(void);
  void reset(void);
  uint8_t writeCtl(uint8_t reg, uint8_t value, char *caller);
  uint8_t readStatus(uint8_t reg, uint8_t *datain, char *caller);
  void enableEventFlags();
};


