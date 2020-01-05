//Rotator4.ino - Mini Satellite-Antenna Rotator.
//Copyright (c) 2015,2016,2017 Julie VK3FOWL and Joe VK3YSP
//Released under the GNU General Public License.
//For more information please visit http://www.sarcnet.org
//Submitted for publication in Amateur Radio magazine: December 2015
//First published in Amateur Radio magazine: May 2016
//Upgraded MkII version published in Amateur Radio magazine: October 2017
//Release history:
//Release 1: Original release
//Release 2: Added support for hamlib 3.0.1. Added debug mode.
//Release 3: Improved calibration and operation
//  Added a low pass filter to the sensor data to improve calibration.
//  Reinitialised the I2C bus and sensor prior to each read to avoid I2C lockups caused by power glitches
//  Changed Serial to Serial1 for RS-422 operation
//Release 4: Improved calibration and operation
//  Fixed a bug reading the EEPROM calibration data on some versions
//  Made the SerialPort configurable to support both the MkI (USB) and MkII (RS422) rotators
//  Added a speaker output to help with the calibration process
//  Removed the overshoot inherent in the anti-windup algorithm
//  Initialised the sensor filters at start up
//  Added a pause command, as requested
//  Added a help menu, as requested
//  Clarified sensor axis definitions, which were confusing
//  Please note that Gpredict position feedback does not work with the USB version on Windows due to a handshaking bug
//  You can fix this by using Linux or the Serial1 port with a TTL to USB converter.

//Includes

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "lsm303d.h"

//Defines

//Please uncomment only one of the following SerialPort definitions:
#define SerialPort Serial     //MkI Rotator: Uses the USB port.
//#define SerialPort Serial1  //MkII Rotator: Uses the TTL port.

#define WINDUP_LIMIT 450  //Sets the total number of degrees azimuth rotation in any direction before resetting to zero

//Types

struct Vector {
  //Vector type comprising three orthogonal components.
  float i, j, k;
};

struct Cal {
  //Calibration data structure
  float md;               //The magnetic declination
  Vector me, ge, ms, gs;  //Calibration vectors
};

enum Modes {tracking, monitoring, demonstrating, calibrating, debugging, pausing};    //Rotator controller modes

//Constants

const float rad2deg = 57.295779513;   //Degrees per radian

//Speaker
const int spkPin = 10;    //Attach a piezo speaker to this pin. It beeps when new calibration data arrives.
const int gndPin = 16;    //Makes a convenient ground pin adjacent to the speaker pin
const int spkFreq = 1800; //Set the beep tone frequency
const int spkTime = 50;   //Set the beep tone time

//Motor drive pins
const int azBrkPin = 4;  //Azimuth break
const int azDirPin = 5;  //Azimuth direction
const int azPwmPin = 6;  //Azimuth pwm
const int elBrkPin = 7;  //Elevation break
const int elDirPin = 8;  //Elevation direction
const int elPwmPin = 9;  //Elevation pwm

//Motor drive gains. These set the amount of motor drive close to the set point
const int azGain = 25;   //Azimuth motor gain
const int elGain = 25;   //Elevation motor gain

//Motor filter constants
const float spdAlpha = 0.4; //Alpha value for speed filter: Decrease to slow response time and reduce motor dither.

//Global variables
float az;               //Antenna azimuth
float el;               //Antenna elevation
String line;            //Command line
float azSet;            //Antenna azimuth set point
float elSet;            //Antenna elevation set point
float azLast;           //Last antenna azimuth reading
float elLast;           //Last antenna element reading
float azWindup;         //Antenna windup angle from startup azimuth position
float azOffset;         //Antenna azimuth offset for whole revolutions
bool windup;            //Antenna windup condition
float azSpeed;          //Antenna azimuth motor speed
float elSpeed;          //Antenna elevation motor speed
float azInc;            //AZ increment for demo mode
float elInc;            //EL increment for demo mode
int t;                  //Variable delay period - depends on mode
Modes mode;             //Rotator mode
Vector mmax, mmin;      //Magnetometer limit vectors
Vector gmax, gmin;      //Accelerometer limit vectors
Cal cal;                //Calibration data
Lsm303d lsm;            //Sensor object

//Functions

//Vector functions

struct Vector cross(Vector a, Vector b) {
  //Returns the vector cross product of two vectors.
  return {a.j * b.k - a.k * b.j, a.k * b.i - a.i * b.k, a.i * b.j - a.j * b.i};
}

float dot(Vector a, Vector b) {
  //Returns the scalar dot product of two vectors.
  return a.i * b.i + a.j * b.j + a.k * b.k;
}

float mod(Vector a) {
  //Returns the scalar modulus of a vector.
  return sqrt(a.i * a.i + a.j * a.j + a.k * a.k);
}

Vector unit(Vector a) {
  //Returns the unit vector of a vector.
  float modulus = mod(a);
  return {a.i / modulus, a.j / modulus, a.k / modulus};
}

Vector neg(Vector a) {
  //Returns the vector negative of a vector.
  return { -a.i, -a.j, -a.k};
}

//Rotator functions

void reset(bool getCal) {
  //Reset the rotator, initialize its variables and optionally get the stored calibration
  azSet = 0.0;
  elSet = 0.0;
  line = "";
  azLast = 0.0;
  elLast = 0.0;
  azWindup = 0.0;
  azOffset = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  mode = tracking;
  windup = false;
  if (getCal) restore();
  mmin = { -1, -1, -1};
  mmax = {1, 1, 1};
  gmin = { -1, -1, -1};
  gmax = {1, 1, 1};
  azInc = 0.05;
  elInc = 0.05;
  t = 100;
}

float diffAngle(float a, float b) {
  //Calculate the acute angle between two angles in -180..180 degree format
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

float lpf(float value, float *last, float alpha) {
  //Low pass filter - Decrease alpha to increase damping factor
  float result = (alpha * value) + *last * (1 - alpha);
  *last = result;
  return result;
}

void drive(float err, int gain, float *spdLast, int brkPin, int dirPin, int pwmPin) {
  //Drive the motors to reduce the antenna pointing errors to zero

  //Calculate the motor speed: A linear ramp close to zero; constant full speed beyond that.
  float spd = constrain(err * gain, -255, 255);

  //Low pass filter the speed to reduce abrubt changes in speed
  spd = lpf(spd, spdLast, spdAlpha);

  //Set the motor direction
  digitalWrite(dirPin, (spd > 0) ? HIGH : LOW);
  //Engage the motor break when the error is small to reduce dither, noise and current consumption
  digitalWrite(brkPin, (abs(err) < 0.5) ? HIGH : LOW);
  //Set the motor speed
  analogWrite(pwmPin, (byte)(abs(spd)));
}

void save() {
  //Save the calibration data to EEPROM
  EEPROM.put(0, cal);
  printCal();
}

void restore() {
  //Restore the calibration data from EEPROM
  EEPROM.get(0, cal);
  printCal();
}

void printDebug(void) {
  //Print raw sensor data
  SerialPort.print(lsm.mx); SerialPort.print(",");
  SerialPort.print(lsm.my); SerialPort.print(",");
  SerialPort.print(lsm.mz); SerialPort.print(",");
  SerialPort.print(lsm.gx); SerialPort.print(",");
  SerialPort.print(lsm.gy); SerialPort.print(",");
  SerialPort.println(lsm.gz);
}

void printCal(void) {
  //Print the calibration data
  SerialPort.print(cal.md, 1); SerialPort.print(",");
  SerialPort.print(cal.me.i, 1); SerialPort.print(",");
  SerialPort.print(cal.me.j, 1); SerialPort.print(",");
  SerialPort.print(cal.me.k, 1); SerialPort.print(",");
  SerialPort.print(cal.ge.i, 1); SerialPort.print(",");
  SerialPort.print(cal.ge.j, 1); SerialPort.print(",");
  SerialPort.print(cal.ge.k, 1); SerialPort.print(",");
  SerialPort.print(cal.ms.i, 1); SerialPort.print(",");
  SerialPort.print(cal.ms.j, 1); SerialPort.print(",");
  SerialPort.print(cal.ms.k, 1); SerialPort.print(",");
  SerialPort.print(cal.gs.i, 1); SerialPort.print(",");
  SerialPort.print(cal.gs.j, 1); SerialPort.print(",");
  SerialPort.println(cal.gs.k, 1);
}

void printMon(float az, float el, float azSet, float elSet, float azWindup, float azError, float elError) {
  //Print the monitor data
  SerialPort.print(az, 0); SerialPort.print(",");
  SerialPort.print(el, 0); SerialPort.print(",");
  SerialPort.print(azSet, 0); SerialPort.print(",");
  SerialPort.print(elSet, 0); SerialPort.print(",");
  SerialPort.print(azWindup, 0); SerialPort.print(",");
  SerialPort.print(windup); SerialPort.print(",");
  SerialPort.print(azError, 0); SerialPort.print(",");
  SerialPort.println(elError, 0);
}

void printAzEl() {
  //Print the rotator feedback data in Easycomm II format
  SerialPort.print("AZ");
  SerialPort.print((az < 0) ? (az + 360) : az, 1);
  SerialPort.print(" EL");
  SerialPort.print(el, 1);
  SerialPort.print("\n");
}

void minmax(float a, float *b, bool *c) {
  //Update exisiting maximums or minimums based on current values
  if ((*b > 0) && (a > *b) || (*b < 0) && (a < *b)) {
    *b = a;
    *c = true;
  }
}

void calibrate() {
  //Caculate the 3D scaling factors and errors for the magnetometer and accelerometer
  bool changed = false;
  //Update exisiting maximums and minimums based on current values
  minmax(lsm.mx, &mmax.i, &changed);
  minmax(lsm.my, &mmax.j, &changed);
  minmax(lsm.mz, &mmax.k, &changed);
  minmax(lsm.gx, &gmax.i, &changed);
  minmax(lsm.gy, &gmax.j, &changed);
  minmax(lsm.gz, &gmax.k, &changed);
  minmax(lsm.mx, &mmin.i, &changed);
  minmax(lsm.my, &mmin.j, &changed);
  minmax(lsm.mz, &mmin.k, &changed);
  minmax(lsm.gx, &gmin.i, &changed);
  minmax(lsm.gy, &gmin.j, &changed);
  minmax(lsm.gz, &gmin.k, &changed);
  //Calculate the error vectors
  cal.me = {(mmax.i + mmin.i) / 2, (mmax.j + mmin.j) / 2, (mmax.k + mmin.k) / 2};
  cal.ge = {(gmax.i + gmin.i) / 2, (gmax.j + gmin.j) / 2, (gmax.k + gmin.k) / 2};
  //Caclulate the scaling vectors
  cal.ms = {(mmax.i - mmin.i) / 2, (mmax.j - mmin.j) / 2, (mmax.k - mmin.k) / 2};
  cal.gs = {(gmax.i - gmin.i) / 2, (gmax.j - gmin.j) / 2, (gmax.k - gmin.k) / 2};
  //Print any changes and beep the speaker to facilitate manual calibration
  if (changed) {
    printCal();                     //Print the calibration data
    tone(spkPin, spkFreq, spkTime); //Beep the speaker
  }
}

void getAzEl(float *az, float *el) {
  //Get the antenna azimuth and elevation angles
  //Get the unit vectors for the earth's magnetic and gravitational fields
  //For each component subtract the error and divide by the scaling factor
  Vector M = unit({(lsm.mx - cal.me.i) / cal.ms.i, (lsm.my - cal.me.j) / cal.ms.j, (lsm.mz - cal.me.k) / cal.ms.k});
  Vector G = unit({(lsm.gx - cal.ge.i) / cal.gs.i, (lsm.gy - cal.ge.j) / cal.gs.j, (lsm.gz - cal.ge.k) / cal.gs.k});
  //Define the antenna axes as the main reference axes
  const Vector X = {1.0, 0.0, 0.0};         //The antenna X vector
  const Vector Y = {0.0, 1.0, 0.0};         //The antenna Y (boresight) vector
  const Vector Z = {0.0, 0.0, 1.0};         //The antenna Z vector
  //Compute the magnetic ground axes relative to the antenna axes
  Vector E = cross(G, M);                   //The magnetic East vector
  Vector N = cross(E, G);                   //The magnetic North vector
  Vector U = neg(G);                        //The magnetic Up vector
  //Compute the projections of the antenna axes onto the magnetic ground axes
  float Xn = dot(X, N);                     //The scalar projection of X onto N
  float Xe = dot(X, E);                     //The scalar projection of X onto E
  float Yu = dot(Y, U);                     //The scalar projection of Y onto U
  float Zu = dot(Z, U);                     //The scalar projection of Z onto U
  //Compute the true antenna pointing angles relative to the magnetic ground axes
  *az = atan2(-Xn, Xe) * rad2deg + cal.md;  //The azimuth angle in degrees using the X-axis
  *el = atan2(Yu, Zu) * rad2deg;            //The elevation angle in degrees using the Y-axis
  if (*az > 180) *az = *az - 360;           //Ensure azimuth is in -180..180 format after adding D
}

void getWindup(bool *windup,  float *azWindup, float *azOffset, float *azLast, float *elLast, float az, float elSet) {
  //Get the accumulated windup angle from the home position (startup or last reset position) and set the windup state if greater than the limit.
  //Get the raw difference angle between the current and last azimuth reading from the sensor
  float azDiff = az - *azLast;

  //Detect crossing South: azDiff jumps 360 for a clockwise crossing or -360 for an anticlockwise crossing
  //Increment the azimuth offset accordingly
  if (azDiff < -180) *azOffset += 360;
  if (azDiff > 180) *azOffset -= 360;

  //Save the current azimuth reading for the next iteration
  *azLast = az;

  //Compute the azimuth wind-up angle, i.e. the absolute number of degrees from the home position
  *azWindup = az + *azOffset;

  //Detect a windup condition where the antenna has rotated more than 450 degrees from home
  if (abs(*azWindup) > WINDUP_LIMIT) *windup = true;    //Set the windup condition - it is reset later when the antenna nears home

  //Perform the anti-windup procedure at the end of each pass - This is overkill unless you absolutely don't want anti-windup during a pass
  //  if (elSet <= 0)
  //    if (elLast > 0)
  //      if (mode == tracking) {
  //        *windup = true;
  //      }

  //Save the current elevation reading for the next iteration
  *elLast = elSet;
}

void getAzElDemo(float *azSet, float *elSet, float *azInc, float *elInc) {
  //Autoincrement the azimuth and elevation to demo the rotator operation
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  SerialPort.print(*azSet, 0); SerialPort.print(",");
  SerialPort.println(*elSet, 0);
}

void getAzElError(float *azError, float *elError, bool *windup, float *azSet, float elSet, float az, float el) {
  //Compute the azimuth and elevation antenna pointing errors, i.e. angular offsets from set positions
  //Compute the azimuth antenna pointing error: Normally via the shortest path; opposite if windup detected.
  if (*windup) {                               //Check for a windup condition
    //To unwind the antenna set an azError in the appropriate direction to home
    *azError = constrain(azWindup, -180, 180); //Limit the maximum azimuth error to -180..180 degrees
    //Cancel the windup condition when the antenna is within 180 degrees of home (Actually 175 degrees to avoid rotation direction ambiguity)
    //Set a zero home position by default, but return azumith control to the computer if still connected
    if (abs(*azError) < 175) *windup = false; //Cancel windup and permit computer control
  }
  else {
    //Compute the normal azimuth antenna pointing error when there is no windup condition
    *azError = diffAngle(az, *azSet);
  }

  //Compute the elevation antenna pointing error
  *elError = diffAngle(el, elSet);
}

void processPosition() {
  //Perform the main operation of positioning the rotator under different modes
  //Read the accelerometer and magnetometer
  lsm.read();
  switch (mode) {
    case debugging:
      printDebug(); //Print the raw sensor data for debug purposes
      break;
    case calibrating:
      calibrate();  //Process calibration data
      break;
    case pausing:
      drive(0, azGain, &azSpeed, azBrkPin, azDirPin, azPwmPin); //Stop the az motor by setting the az error to zero
      drive(0, elGain, &elSpeed, elBrkPin, elDirPin, elPwmPin); //Stop the el motor by setting the el error to zero
      break;
    default:
      getAzEl(&az, &el);                                                          //Get the antenna AZ and EL
      getWindup(&windup, &azWindup, &azOffset, &azLast, &elLast, az, elSet);      //Get the AZ windup angle and windup state
      if (mode == demonstrating) getAzElDemo(&azSet, &elSet, &azInc, &elInc);     //Set the AZ and EL automatically if in demo mode
      float azError, elError;
      getAzElError(&azError, &elError, &windup, &azSet, elSet, az, el);           //Get the antenna pointing error
      if (mode == monitoring) printMon(az, el, azSet, elSet, azWindup, azError, elError); //Print the data if in monitor mode
      //Drive the motors to reduce the azimuth and elevation error to zero
      drive(azError, azGain, &azSpeed, azBrkPin, azDirPin, azPwmPin);
      drive(elError, elGain, &elSpeed, elBrkPin, elDirPin, elPwmPin);
      break;
  }
}

void processUserCommands(String line) {
  //Process user commands
  //User command type 1: r, b, m, c, a, d, s, d, h, p or e<decl> followed by a carriage return
  //User command type 2: <az> <el> followed by a carriage return
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  int secondSpace;                                        //Position of the second space in the command line
  char command = line.charAt(0);                          //Get the first character
  switch (command) {                                      //Process type 1 user commands
    case 'r':                                             //Reset command
      SerialPort.println("Reset in progress");
      reset(true);  //Reset the rotator and load calibration from EEPROM
      SerialPort.println("Reset");
      break;
    case 'b':                                             //Debug command
      SerialPort.println("Debugging in progress: Press 'a' to abort");
      mode = debugging;
      t = 100;
      break;
    case 'm':                                             //Monitor command
      SerialPort.println("Monitoring in progress: Press 'a' to abort");
      mode = monitoring;
      t = 100;
      break;
    case 'c':                                             //Calibrate command
      SerialPort.println("Calibration in progress: Press 'a' to abort or 's' to save");
      reset(false); //Reset the rotator, but don't load calibration from EEPROM
      mode = calibrating;
      t = 50;
      break;
    case 'a':                                             //Abort command
      mode = tracking;
      t = 100;
      reset(true);
      SerialPort.println("Function aborted");
      break;
    case 'e':                                             //Magnetic declination command
      param = line.substring(1);                          //Get the second parameter
      cal.md = param.toFloat();
      break;
    case 's':                                             //Save command
      save();
      reset(true);
      SerialPort.println("Calibration saved");
      break;
    case 'd':                                             //Demo command
      SerialPort.println("Demo in progress: Press 'a' to abort");
      t = 25;
      mode = demonstrating;
      break;
    case 'h':                                             //Help command
      SerialPort.println("Commands:");
      SerialPort.println("az el-(0..360 0..90)");
      SerialPort.println("r-Reset");
      SerialPort.println("eNN.N-MagDecl");
      SerialPort.println("c-Calibrate");
      SerialPort.println("s-Save");
      SerialPort.println("a-Abort");
      SerialPort.println("d-Demo");
      SerialPort.println("b-Debug");
      SerialPort.println("m-Monitor");
      SerialPort.println("p-Pause");
      break;
    case 'p':                                             //Pause command
      if (mode == pausing) {
        mode = tracking;
      } else {
        mode = pausing;
        SerialPort.println("Paused");
      }

      break;
    default:                                              //Process type 2 user commands
      firstSpace = line.indexOf(' ');                     //Get the index of the first space
      param = line.substring(0, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Get the azSet value
      param = line.substring(firstSpace + 1);             //Get the second parameter
      elSet = param.toFloat();                            //Get the elSet value
  }
}

void processEasycommCommands(String line) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \n
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  int secondSpace;                                        //Position of the second space in the command line
  if (line.startsWith("AZ EL")) {                         //Query command received
    printAzEl();                                          //Send the current Azimuth and Elevation
  } else {
    if (line.startsWith("AZ")) {                          //Position command received: Parse the line.
      firstSpace = line.indexOf(' ');                     //Get the position of the first space
      secondSpace = line.indexOf(' ', firstSpace + 1);    //Get the position of the second space
      param = line.substring(2, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Set the azSet value
      if (azSet > 180) azSet = azSet - 360;               //Convert 0..360 to -180..180 degrees format
      param = line.substring(firstSpace + 3, secondSpace);//Get the second parameter
      elSet = param.toFloat();                            //Set the elSet value
    }
  }
}

void processCommands(void) {
  //Process incoming data from the control computer
  //User commands are entered by the user and are terminated with a carriage return
  //Easycomm commands are generated by a tracking program and are terminated with a line feed
  while (SerialPort.available() > 0) {
    char ch = SerialPort.read();                                //Read a single character from the serial buffer
    switch (ch) {
      case 13:                                                  //Carriage return received
        processUserCommands(line);                              //Process user commands
        line = "";                                              //Command processed: Clear the command line
        break;
      case 10:                                                  //Line feed received
        processEasycommCommands(line);                          //Process Easycomm commands
        line = "";                                              //Command processed: Clear the command line
        break;
      default:                                                  //Any other character received
        line += ch;                                             //Add this character to the command line
        break;
    }
  }
}

void setup() {
  //Initialize the system
  //Set speaker pins to outputs
  pinMode(spkPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);
  //Set motor pins to outputs
  pinMode(azBrkPin, OUTPUT);
  pinMode(azDirPin, OUTPUT);
  pinMode(azPwmPin, OUTPUT);
  pinMode(elBrkPin, OUTPUT);
  pinMode(elDirPin, OUTPUT);
  pinMode(elPwmPin, OUTPUT);
  //Reset the rotator and load configuration from EEPROM
  reset(true);
  //Initialize the serial port
  SerialPort.begin(9600);
  //Initialize the sensor
  lsm.begin();
}

void loop() {
  //Repeat continuously
  processCommands();                                            //Process commands from the control computer
  processPosition();                                            //Process position feedback from the sensors
  delay(t);                                                     //Wait an appropriate time depending on the mode
}

