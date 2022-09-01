/*
  QMC5883L.cpp - QMC5883L library
  Copyright (c) 2017 e-Gizmo Mechatronix Central
  Rewritten by Amoree.  All right reserved.
  July 10,2017

  SET continuous measurement mode
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
 
*/

#include "QMC5883L.h"

#include <Wire.h>

void QMC5883L::setAddress(uint8_t addr){
  address = addr;
}

void QMC5883L::WriteReg(byte Reg,byte val){
  Wire.beginTransmission(address); //Start
  Wire.write(Reg); // To tell the QMC5883L to get measures continously
  Wire.write(val); //Set up the Register
  Wire.endTransmission();
}

void QMC5883L::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
}

void QMC5883L::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
  WriteReg(0x09,mode|odr|rng|osr);
  Serial.println(mode|odr|rng|osr,HEX);
}


void QMC5883L::softReset(){
  WriteReg(0x0A,0x80);
}

void QMC5883L::read(int* x,int* y,int* z){
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  *x = Wire.read(); //LSB  x
  *x |= Wire.read() << 8; //MSB  x
  *y = Wire.read(); //LSB  z
  *y |= Wire.read() << 8; //MSB z
  *z = Wire.read(); //LSB y
  *z |= Wire.read() << 8; //MSB y
}
