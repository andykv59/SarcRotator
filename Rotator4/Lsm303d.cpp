//Lsm303d.cpp - Library for LSM303D Accelerometer/Magnetometer.
//Copyright (c) 2015,2016,2017 Julie VK3FOWL and Joe VK3YSP
//Released under the GNU General Public License.
//For more information please visit http://www.sarcnet.org
//Submitted for publication in Amateur Radio magazine: December 2015
//First published in Amateur Radio magazine: May 2016
//Upgraded MkII version published in Amateur Radio magazine: October 2016
//Reference: ST Datasheet: LSM303D Ultra-compact high-performance eCompass module: 3D accelerometer and 3D magnetometer
//The following clarification is required to rationalize the vector diagram and sensor axes and in the original article with the actual LSM303D device axes.
//The LSM303D device axes (x', y' and z') are not the same as the (x, y and z) axes in the vector diagram.
//x' points along the boom, y' points to the left, z' points up. So: x = -y', y = x' and z = z'.
//Also, in the steady state, the gravity field vector g is the opposite of the device acceleration vector a: Acceleration points up, gravity points down. There is no difference in the magnetic field vector m: It points from the Earth's South magnetic pole to the North.
//Therefore the following transformations apply: mx = -my', my = mx', mz = mz', gx = ay', gy = -ax', gz = -az'.//Release 4
//Release 4

#include "Lsm303d.h"

//Constructor
void Lsm303d::begin() {
  //Initialize the Lsm303d object
  //Reset the sensor
  reset();
  //Initialize the sensor filters
  for (int i = 0; i < 50; i++) read();
}

//Public

void Lsm303d::reset() {
  //Reset the sensor
  Wire.begin();                                         //Initialize the I2C bus
  writeReg(LSM303D_ADDRESS, LSM303D_CTRL1, 0b01010111); //Acc output data rate = 50Hz all Acc axes enabled.
  writeReg(LSM303D_ADDRESS, LSM303D_CTRL2, 0b00000000); //Acc full scale = +/- 2g
  writeReg(LSM303D_ADDRESS, LSM303D_CTRL5, 0b01100100); //Mag output data rate = 6.25HzMag resolution = high;
  writeReg(LSM303D_ADDRESS, LSM303D_CTRL6, 0b00100000); //Mag full scale = +/- 4gauss
  writeReg(LSM303D_ADDRESS, LSM303D_CTRL7, 0b00000000); //Mag low power mode = Off. Mag sensor mode = Continuous-conversion
}

void Lsm303d::read() {
  //Read the accelerometer and magnetometer
  reset();  //Reinitialise I2C and the sensor before each read as it is inclined to lock up after running a long time
  readM();
  readG();
}

//Private

int Lsm303d::lpf(int value, int *last, float alpha) {
  //Low pass filter - Decrease alpha to increase damping factor
  int result = (alpha * value) + *last * (1 - alpha);
  *last = result;
  return result;
}

void Lsm303d::readG() {
  //Read the 3D accelerometer to determine the gravitational field vector
  Wire.beginTransmission((byte)LSM303D_ADDRESS);
  Wire.write(LSM303D_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303D_ADDRESS, (byte)6);
  if (Wire.available() == 6) {
    //Read 8-bit values
    byte  xl = Wire.read();
    byte  xh = Wire.read();
    byte  yl = Wire.read();
    byte  yh = Wire.read();
    byte  zl = Wire.read();
    byte  zh = Wire.read();
    //Assemble 16-bit values and perform the axis transformation
    gx = ((yh << 8) | yl);
    gy = -((xh << 8) | xl);
    gz = -((zh << 8) | zl);
    //Low pass filter the sensor data as it improves the calibration procedure
    gx = lpf(gx, &gxLast, alpha);
    gy = lpf(gy, &gyLast, alpha);
    gz = lpf(gz, &gzLast, alpha);
  }
}

void Lsm303d::readM() {
  //Read the 3D manetometer to determine the magnetic field vector
  Wire.beginTransmission((byte)LSM303D_ADDRESS);
  Wire.write(LSM303D_OUT_X_L_M | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303D_ADDRESS, (byte)6);
  if (Wire.available() == 6) {
    //Read 8-bit values
    byte xl = Wire.read();
    byte xh = Wire.read();
    byte yl = Wire.read();
    byte yh = Wire.read();
    byte zl = Wire.read();
    byte zh = Wire.read();
    //Assemble 16-bit values and perform the axis transformation
    mx = -((yh << 8) | yl);
    my = ((xh << 8) | xl);
    mz = ((zh << 8) | zl);
    //Low pass filter the sensor data as it improves the calibration procedure
    mx = lpf(mx, &mxLast, alpha);
    my = lpf(my, &myLast, alpha);
    mz = lpf(mz, &mzLast, alpha);
  }
}

void Lsm303d::writeReg(byte address, byte reg, byte value) {
  //I2C write to register at address
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte Lsm303d::readReg(byte address, byte reg) {
  //I2C read from register at address
  byte value;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

