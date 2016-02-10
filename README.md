# Arduino MPL3115A2 altitude/barometric/temperature sensor control over I2C

This .cpp and .h file provide a library of procedures for collecting readings from 
the Freescale MPL3115A2 altitude/barometric pressure/temperature sensor.

This library uses Truchsess's I2C library for communication with the MPL3115A2 to avoid
the misaligned data caused by repeated-starts that occurred with the older Wire library.
Sampling was found to be unreliable over time when using the Wire library, <wire.h>
Converted to use I2C library since it handles the repeated START I2C commands that the 
Freescale, Inc., devices use.

Thanks to Wayne Truchess for his excellent work on the I2C system.
See http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library

Based on code by: A.Weiss, 7/17/2012, changes Nathan Seidle Sept 23rd, 2013 (SparkFun)
License: This code is public domain but you buy me (Seidle) a beer if you use this 
and we meet someday (Beerware license).  
Revisions for I2C by HDTodd, July, 2015
 
Hardware Connections to MPL3115A2 (Breakout board to Arduino):
* VCC = 3.3V
* SDA = A4		Add 330 ohm resistor in series if using 5v VCC
* SCL = A5		Add 330 ohm resistor in series if using 5v VCC
* GND = GND
* INT pins can be left unconnected for this demo
 
Usage:
* Serial terminal at 9600bps
* Times various sensor measurements
*-Examines status flags used to poll device for data ready
 
During testing, GPS with 9 satellites reported 5393ft, sensor reported 5360ft 
(delta of 33ft). Very close! (Seidle)
During testing, GPS with 9 satellites reported 1520m; sensor reported 1433m; 
surprisingly far off (Todd)  [So I chose to tell it what the altitude is and 
have it apply the correction -- Todd]

