#include "Gcode.h"

double Gcode::prevAcceleration = 0; // mm/s^2
double Gcode::prevFeedSpeed = 0; // mm/min
double Gcode::prevX = 0; // mm
double Gcode::prevY = 0; // mm
double Gcode::prevZ = 0; // mm
double Gcode::prevE = 0; // mm

Gcode::Gcode(){
}

Gcode::Gcode(String cmdStr){
}

void Gcode::setGcodeArg(char flag, String num) {
  switch (flag)
  {
  case 'G':
    cmdtype = 'G';
    cmdnum = num.toInt();
    break;
  case 'M':
    cmdtype = 'M';
    cmdnum = num.toInt();
    break;
  case 'T':
    T = num.toFloat();
    // cmdtype = 'T';
    // cmdnum = num.toInt();
    break;
  case 'X':
    X = num.toFloat();
    break;
  case 'Y':
    Y = num.toFloat();
    break;
  case 'Z':
    Z = num.toFloat();
    break;
  case 'E':
    E = num.toFloat();
    break;
  case 'F':
    F = num.toFloat();
    break;
  case 'S':
    S = num.toFloat();
    break;
  case 'P':
    P = num.toFloat();
    break;
  case 'I':
    I = num.toFloat();
    break;
  case 'D':
    D = num.toFloat();
    break;
  
  default:
    break;
  }
}

Gcode Gcode::parse(String cmdStr)
{
  Gcode gcode = {};
  gcode.hasX = false;
  gcode.hasY = false;
  gcode.hasZ = false;
  gcode.hasE = false;
  gcode.hasF = false;
  gcode.hasS = false;

  int commentIndex = cmdStr.indexOf(';');
  if (commentIndex)
    cmdStr = cmdStr.substring(0, commentIndex);

  // int cmdnum = -1;
  String cache = "";
  char curParse = '1';
  bool skip = false;
  for (uint8_t i = 0; i < cmdStr.length(); i++)
  {
    char cur = cmdStr.charAt(i);
    switch (cur)
    {
    case 'G':
      curParse = 'G';
      skip = true;
      break;
    case 'M':
      curParse = 'M';
      skip = true;
      break;
    case 'T':
      curParse = 'T';
      skip = true;
      break;
    case 'X':
      curParse = 'X';
      gcode.hasX = true;
      skip = true;
      break;
    case 'Y':
      curParse = 'Y';
      gcode.hasY = true;
      skip = true;
      break;
    case 'Z':
      curParse = 'Z';
      gcode.hasZ = true;
      skip = true;
      break;
    case 'E':
      curParse = 'E';
      gcode.hasE = true;
      skip = true;
      break;
    case 'F':
      curParse = 'F';
      gcode.hasF = true;
      skip = true;
      break;
    case 'I':
      curParse = 'I';
      skip = true;
      break;
    case 'J':
      curParse = 'J';
      skip = true;
      break;
    case 'R':
      curParse = 'R';
      skip = true;
      break;
    case 'S':
      curParse = 'S';
      gcode.hasS = true;
      skip = true;
      break;
    case 'P':
      curParse = 'P';
      skip = true;
      break;
    case 'D':
      curParse = 'D';
      skip = true;
      break;
    
    case ' ':
      gcode.setGcodeArg(curParse, cache);
      skip = true;
      cache = "";
      break;
    default:
      skip = false;
      break;
    }
    if (i == cmdStr.length() - 1 && cur != ' ')
    {
      cache.concat(cur);
      gcode.setGcodeArg(curParse, cache);
      cache = "";
    }

    if (!skip)
    {
      cache.concat(cur);
    }
  }

  return gcode;
}