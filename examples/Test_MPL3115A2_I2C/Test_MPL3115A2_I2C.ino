/*
  Test_MPL3115A2.ino
  This program for the Arduino (Uno) tests the library of procedures for
  collecting readings from the Freescale MPL3115A2 altitude/barometric pressure/temperature
  sensor.

  This library uses Truchsess's I2C library for communication with the MPL3115A2 to avoid
  the misaligned data bits caused by repeated-starts with the older Wire library.

  Sampling found to be unreliable over time when using the Wire library, <wire.h>
  Converted to use I2C library since it handles the repeated START I2C commands that the 
  Freescale, Inc., devices use.

  Thanks to Wayne Truchsess for his excellent work on that system.
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

#define MY_ALTITUDE 1520           // Set this to your actual GPS-verified altitude in meters if needed

uint8_t sampleRate=S_1;            // We'll cycle through all 8 settings, but start with 1 sample/reading
MPL3115A2 thing;                   // This is the thing we'll be sampling
      
void setup()
{
  Serial.begin(9600);   // start serial for output
  I2c.begin();          // join i2c bus
  I2c.setSpeed(1);      // go fast

  thing = MPL3115A2();  // create one

  if (!thing.begin()) {
    Serial.println("Can't init MPL3115A2!");
    delay(1000);
    exit(1);
  }

  thing.setAltitude(MY_ALTITUDE);       // try with MY_ALTITUDE set to 0 and see if altimeter is
                                        // accurate; if not, set MY_ALTITUDE to GPS-provided altitude in meters

// And announce ourselves
  Serial.println("Characterizing MPL3115A2 performance on an Arudino with access via I2C library"); 
  Serial.println("Reporting value read and time to read (in msec) for 1..128 oversample counts");
}

void loop()
{
  float altitude, pressure, temperature;
  long startTime, inter;
  int altTime, pressTime, tempTime, loopTime;
  uint8_t count;
  
  thing.setOversampleRate(sampleRate++);      // Set the oversample rate for this set of readings,
                                              //   and cycle through all 8 possible settings

  startTime = millis();
  altitude = thing.readAltitude();
  altTime = (int) ( ( inter=millis() ) - startTime );
  
  pressure = thing.readPressure();
  pressTime = (int) (millis() - inter);

  inter = millis();
  temperature = thing.readTemp();
  tempTime = (int) (millis() - inter);
  
  Serial.print("SampleCount = ");             // Tell how many counts/sample
  count = 1<<thing.getOversampleRate();
  if (count < 10) Serial.print(" "); if (count<100) Serial.print(" ");
  Serial.print(count);
  Serial.print("   "); 
   
  Serial.print("Altitude: ");                 // Tell altitude & timing
  Serial.print(altitude, 1);
  Serial.print(" m [");
  if (altTime < 10) Serial.print(" "); if (altTime<100) Serial.print(" ");
  Serial.print(altTime);
  Serial.print("ms]   ");

  Serial.print("Pressure: ");                 // Tell pressure & timing
  Serial.print(pressure, 0);
  Serial.print(" Pa [");
  if (pressTime < 10) Serial.print(" "); if (pressTime<100) Serial.print(" ");
  Serial.print(pressTime);
  Serial.print("ms]   ");

  Serial.print("Temp: ");                     // Tell temperature & timing
  Serial.print(temperature*1.80+32.0, 1);
  Serial.print(" F = ");
  Serial.print(temperature,1);
  Serial.print(" C [");
  if (tempTime < 10) Serial.print(" "); if (tempTime<100) Serial.print(" ");
  Serial.print(tempTime);
  Serial.print("ms]   ");

  loopTime = millis() - startTime;            // Tell overall loop time (incl prints)
  Serial.print("Loop time: ");
  if (loopTime < 10) Serial.print(" "); if (loopTime<100) Serial.print(" ");
  if (loopTime < 1000) Serial.print(" ");  
  Serial.print(loopTime);
  Serial.println(" ms");

  sampleRate %= 8;                            // repeat 0..7 in successive loops
  delay(5000);
}
