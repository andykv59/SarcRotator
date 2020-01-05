//mot.cpp - Motor library.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//For more information please visit http://www.sarcnet.org
//Released under the GNU General Public License.
//Provides DC motor speed and direction control
#include "mot.h"
#include "arduino.h"

//Public methods

Mot::Mot(float alpha, int gain, int brkPin, int dirPin, int pwmPin): fil(alpha) {
  //Constructor
  _gain = gain;
  _brkPin = brkPin;
  _dirPin = dirPin;
  _pwmPin = pwmPin;
  pinMode(_brkPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
}

Mot::Mot(float alpha, int gain, int fwdPin, int revPin): fil(alpha) {
  //Constructor
  _gain = gain;
  _fwdPin = fwdPin;
  _revPin = revPin;
  pinMode(_fwdPin, OUTPUT);
  pinMode(_revPin, OUTPUT);
}

Mot::drive(float err) {
  //Drive the motors to reduce the antenna pointing errors to zero
  //Calculate the motor speed: A linear ramp close to zero; constant full speed beyond that.
  float spd = constrain(err * _gain, -255, 255);
  //Low pass filter the speed to reduce abrubt changes in speed
  spd = fil.lpf(spd);

#ifdef PWMDIR
  //For LMD18200T DC Motor H-Bridge Driver Boards
  //Set the motor direction
  digitalWrite(_dirPin, (spd > 0) ? HIGH : LOW);
  //Engage the motor break when the error is small to reduce dither, noise and current consumption
  digitalWrite(_brkPin, (abs(err) < 0.5) ? HIGH : LOW);
  //Set the motor speed
  analogWrite(_pwmPin, (byte)(abs(spd)));
#endif
#ifdef FWDREV
  //For L298N DC Motor H-Bridge Driver Boards
  //Set the motor speed
  if (abs(err) < 0.5) {
    analogWrite(_fwdPin, 0);
    analogWrite(_revPin, 0);
  } else {
    if (spd > 0) {
      analogWrite(_fwdPin, (byte)(spd));
      analogWrite(_revPin, 0);
    } else {
      analogWrite(_fwdPin, 0);
      analogWrite(_revPin, (byte)(-spd));
    }
  }
#endif
}

Mot::halt() {
  drive(0.0);
}

