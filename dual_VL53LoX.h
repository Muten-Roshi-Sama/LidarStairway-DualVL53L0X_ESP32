#ifndef DUAL_VL53LOX_H
#define DUAL_VL53LOX_H

#include "Adafruit_VL53L0X.h"

// Sensor Addresses
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Sensor Shutdown Pins
#define SHT_LOX1 6
#define SHT_LOX2 7

// Sensor Objects
// extern Adafruit_VL53L0X lox1;
// extern Adafruit_VL53L0X lox2;

// Measurement Data
extern int sensor1Data;
extern int sensor2Data;

// Function Prototypes
void init_VL53Lox();
void setID();
void read_dual_sensors();

#endif // DUAL_VL53LOX_H
