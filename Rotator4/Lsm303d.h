//Lsm303d.h - Library for LSM303D Accelerometer/Magnetometer.
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

#ifndef LSM303D_H
#define LSM303D_H
#include <Arduino.h>
#include <Wire.h>
#define LSM303D_ADDRESS 0b0011101
#define LSM303D_OUT_X_L_A 0x28
#define LSM303D_OUT_X_L_M 0x08
#define LSM303D_CTRL1 0x20
#define LSM303D_CTRL2 0x21
#define LSM303D_CTRL3 0x22
#define LSM303D_CTRL4 0x23
#define LSM303D_CTRL5 0x24
#define LSM303D_CTRL6 0x25
#define LSM303D_CTRL7 0x26

class Lsm303d {
  public:
    int gx, gy, gz, mx, my, mz;
    void begin(void);
    void read(void);
  private:
    const float alpha = 0.1;  //Low pass filter damping factor - Decrease to increase damping
    int gxLast, gyLast, gzLast, mxLast, myLast, mzLast;
    int lpf(int value, int *last, float alpha);
    void reset();
    void readG();
    void readM();
    void writeReg(byte address, byte reg, byte value);
    byte readReg(byte address, byte reg);
};
#endif

