// acceleroMMA7361.h - Library for retrieving data from the MMA7361 accelerometer. 
// Copyright 2011-2012 Jef Neefs (neefs@gmail.com) and Jeroen Doggen (jeroendoggen@gmail.com)
// Datasheet: http://www.sparkfun.com/datasheets/Components/General/MMA7361L.pdf
//
// Version History:
//  Version 0.1: -get raw values
//  Version 0.2: -get voltages and G forces
//  Version 0.3: -removed begin parameters offset
//               -added public function setOffSets(int,int,int)
//               -added a private variable _offSets[3] containing the offset on each axis
//               -changed long and double return values of private and public functions to int
//  Version 0.4: -added calibrate
//  Version 0.5: -added setARefVoltage
//               -added setAveraging
//               -added a default begin function
//  Version 0.6: -added getAccelXYZ to get all axis in one call
//               -added getTotalVector returns the magnitude of the total vector as an integer
//               -added getOrientation returns which axis perpendicular with the earths surface x=1,y=2,z=3
//                is positive or negative depending on which side of the axis is pointing downwards
//  Version 0.7: -added setSensitivity
//               -added sleep & wake
//  Version 0.8: -converted to Arduino 1.0 library
//               -changed license to LGPL
// Roadmap:
//  Version 0.x: auto zero calibration http://www.freescale.com/files/sensors/doc/app_note/AN3447.pdf
//  Version 0.x: We asumed the output to be linear, it is nearly linear but not exectly...
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef AcceleroMMA7361_h
#define AcceleroMMA7361_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

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
    void calibrate();                             // only to be executed when Z-axis is oriented to the ground
// it calculates the offset values by assuming  Z = +1 G ; X and Y  = 0 G
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
