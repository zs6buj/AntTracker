/*
  QMC5883L.h - QMC5883L library
  Copyright (c) 2017 e-Gizmo Mechatronix Central
  Rewritten by Amoree.  All right reserved.
  July 10,2017
*/

#ifndef QMC5883L_h
#define QMC5883L_h


#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Wire.h"

#define QMC5883L_ADDR 0x0D//The default I2C address is 0D: 0001101


//Registers Control //0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


class QMC5883L{

public:
void setAddress(uint8_t addr);
void init();
void setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr);
void softReset();
void read(uint16_t* x,uint16_t* y,uint16_t* z);
void read(int* x,int* y,int* z);

private:
void WriteReg(uint8_t Reg,uint8_t val);
uint8_t address = QMC5883L_ADDR;

};

#endif
