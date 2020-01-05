//mot.h - Motor library.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//For more information please visit http://www.sarcnet.org
//Released under the GNU General Public License.
//Provides DC motor speed and direction control
#ifndef MOT_H
#define MOT_H
#include "fil.h"

//Please uncomment only one of the following to select the LMD18200T or L298N driver boards
#define PWMDIR   //For single PWM output with direction and break control. E.g. LMD18200T
//#define FWDREV   //For dual forward and reverse PWM output with no direction and break control. E.g. L298N

class Mot {
  public:
    Mot(float alpha, int gain, int brkPin, int dirPin, int pwmPin);
    Mot(float alpha, int gain, int fwdPin, int revPin);
    drive(float err);
    halt();
  private:
    int _gain, _brkPin,  _dirPin,  _pwmPin, _fwdPin,  _revPin;
    Fil fil;
};
#endif
