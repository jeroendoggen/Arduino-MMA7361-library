/****************************************************************************************************************
 * acceleroMMA7361.h - Library for retrieving data from the MMA7361 accelerometer. Only tested with Duemilanove *
 * DATASHEET: http://www.sparkfun.com/datasheets/Components/General/MMA7361L.pdf                                *
 * Following functions need improving. I asumed the output to be linear (it is nearly linear but not really).   *
 * begin variables                                                                                              *
 *  -int sleepPin: number indicating to which pin the sleep port is attached.       DIGITAL OUT                 *
 *  -int selfTestPin: number indicating to which pin the selftest port is attached. DIGITAL OUT                 *
 *  -int zeroGPin: number indicating to which pin the ZeroGpin is connected to.     DIGITAL IN                  *
 *  -int gSelectPin: number indication to which pin the Gselect is connected to.      DIGITAL OUT               *
 *  -int xPin: number indicating to which pin the x-axis pin is connected to.       ANALOG IN                   *
 *  -int yPin: number indicating to which pin the y-axis  pin is connected to.      ANALOG IN                   *
 *  -int zPin: number indicating to which pin the z-axis  pin is connected to.      ANALOG IN                   *
 *  -int offset: array indicating the G offset on the x,y and z-axis                                            *
 * When you use begin() without variables standard values are loaded:A0,A1,A2 as input for X,Y,Z                *
 *                                         and digital pins 13,12,11,10 for sleep, selftest, zeroG and gSelect  *
 * Functions currently present:                                                                                 *
 *  -getXRaw(): Returns the raw data from the X-axis analog I/O port of the Arduino as an integer               *
 *  -getYRaw(): Returns the raw data from the Y-axis analog I/O port of the Arduino as an integer               *
 *  -getZRaw(): Returns the raw data from the Z-axis analog I/O port of the Arduino as an integer               *
 *  -getXVolt(): Returns the voltage in miliVolts from the X-axis analog I/O port of the Arduino as a integer   *
 *  -getYVolt(): Returns the voltage in miliVolts from the Y-axis analog I/O port of the Arduino as a integer   *
 *  -getZVolt(): Returns the voltage in miliVolts from the Z-axis analog I/O port of the Arduino as a integer   *
 *  -getXAccel(): Returns the acceleration of the X-axis as a int (1G = 100.00)                                 *
 *  -getYAccel(): Returns the acceleration of the Y-axis as a int (1G = 100.00)                                 *
 *  -getXAccel(): Returns the acceleration of the Z-axis as a int (1G = 100.00)                                 *
 *  -setOffSets( int offSetX, int offSetY, int offSetZ): Sets the offset values for the x,y & z  axxis.         *
 *    The parameters are the offsets expressed in G-force (100 = 1G) offsets are added to the raw datafunctions *
 *  -calibrate(): Sets X and Y values via setOffsets to zero. The Z axis will be set to 100 = 1G               *
 *    WARNING WHEN calibrateD YOU HAVE TO SEE THE Z-AXIS IS PERPENDICULAR WITH THE EARTHS SURFACE              *
 *  -setARefVoltage(double _refV): Sets the AREF voltage to external, ( now only takes 3.3 or 5 as parameter)   *
 *    default is 5 when no AREF is used. When you want to use 3.3 AREF, put a wire between the AREF pin and the *
 *    3.3V VCC pin. This increases accuracy                                                                     *
 *  -setAveraging(int avg): Sets how many samples have to be averaged in getAccel default is 10                 *
 *  -getAccelXYZ(int *_XAxis, int *_YAxis, int *_ZAxis) returns all axis at once as pointers                    *
 *  -getTotalVector returns the magnitude of the total vector as an integer                                     *
 *  -getOrientation returns which axis perpendicular with the earths surface x=1,y=2,z=3 is positive or         *
 *     negative depending on which side of the axis is pointing downwards                                       *
 *  -setSensitivity sets the sensitivity to +/-6G or +/-1.5G by a boolean HIGH (1.5) or LOW (6)                 *
 *  -sleep lets the device sleep (when device is sleeping already this does nothing)                            *
 *  -wake enables the device after sleep (when device is not sleeping this does nothing) there is a 2ms delay   *
 *     due to enable time specifications by the datasheet                                                       *
 ****************************************************************************************************************
 * Version History:                                                                                             *
 *  Version 0.1: -get raw values                                                                                *
 *  Version 0.2: -get voltages and G forces                                                                     *
 *  Version 0.3: -removed begin parameters offset                                                               *
 *               -added public function setOffSets(int,int,int)                                                 *
 *               -added a private variable _offSets[3] containing the offset on each axis                       *
 *               -changed long and double return values of private and public functions to int                  *
 *  Version 0.4: -added calibrate                                                                              *
 *  Version 0.5: -added setARefVoltage                                                                          *
 *               -added setAveraging                                                                            *
 *               -added a default begin function                                                                *
 *  Version 0.6: -added getAccelXYZ to get all axis in one call                                                 *
 *               -added getTotalVector returns the magnitude of the total vector as an integer                  *
 *               -added getOrientation returns which axis perpendicular with the earths surface x=1,y=2,z=3     *
 *                is positive or negative depending on which side of the axis is pointing downwards             *
 *  Version 0.7: -added setSensitivity                                                                          *
 *               -added sleep & wake                                                                            *
 *  todo implement auto zero calibration http://www.freescale.com/files/sensors/doc/app_note/AN3447.pdf         *
 ****************************************************************************************************************
 * Created by Jef Neefs: Suggestions, questions or comments please contact me                                   *
 *  -mail: neefsj at gmail dot com                                                                              *
 *  -skype: studioj                                                                                             *
 ***************************************************************************************************************/
#ifndef AcceleroMMA7361_h
#define AcceleroMMA7361_h
#include <WProgram.h>

class AcceleroMMA7361
{
public:
  AcceleroMMA7361();
  void begin();
  void begin(int sleepPin, int selfTestPin, int zeroGPin, int gSelectPin, int xPin, int yPin, int zPin);
  int getXRaw();
  int getYRaw();
  int getZRaw();
  int getXVolt();
  int getYVolt();
  int getZVolt();
  int getXAccel();
  int getYAccel();
  int getZAccel();
  void getAccelXYZ(int *_XAxis, int *_YAxis, int *_ZAxis);
  int getTotalVector();
  void setOffSets(int xOffSet, int yOffSet, int zOffSet);
  void calibrate();            // only to be executed when Z-axxis is orientated to the ground 
                                // it calculates the offset values by asuming  Z = +1G ; X and Y  = 0G
  void setARefVoltage(double _refV);
  void setAveraging(int avg);
  int getOrientation();
  void setSensitivity(boolean sensi);
  void sleep();
  void wake();

private:
  int _mapMMA7361V(int value);
  int _mapMMA7361G(int value);
  int _sleepPin;
  int _selfTestPin;
  int _zeroGPin;
  int _gSelectPin;
  int _xPin;
  int _yPin;
  int _zPin;
  int _offSets[3];
  int _polarities[3];
  double _refVoltage;
  int _average;
  boolean _sleep;
  boolean _sensi;
};
#endif