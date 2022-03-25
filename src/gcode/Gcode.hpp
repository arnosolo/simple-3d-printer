#ifndef _GCODE_HPP_
#define _GCODE_HPP_
#include <Arduino.h>
#include "Utils/Queue.hpp"
#include "config/config.hpp"

class Gcode
{
private:
public:
  char cmdtype = '1';
  int cmdnum = -1;
  double X= 0;
  double Y= 0;
  double Z= 0;
  double E= 0;
  double F= 0;
  double T= 0;
  double S= 0;
  double P= 0;
  double I= 0;
  double D= 0;
  bool hasX;
  bool hasY;
  bool hasZ;
  bool hasE;
  bool hasF;
  bool hasS;
  bool hasP;
  bool hasI;
  bool hasD;

  static double prevAcceleration; // mm/s^2
  static double prevFeedSpeed; // mm/min
  static double prevX; // mm
  static double prevY; // mm
  static double prevZ; // mm
  static double prevE; // mm
  static Queue<String> strQueue;

  Gcode();
  Gcode(String cmdStr);
  // ~Gcode();
  void setGcodeArg(char flag, String num);

  static Gcode parse(String cmdStr);

  static void M104(Gcode* gcode);
};

#endif 