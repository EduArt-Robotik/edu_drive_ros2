#pragma once

#include "CommandMultiplexer.h"

#include "sensor_msgs/msg/joy.hpp"

namespace edu {

class JoystickInputHandler {
public:
  struct JoystickMap {
    struct Buttons {
      int omniMode = 11;
      int servoLeft = 2;
      int servoRight = 3;
      int disable = 9;
      int enable = 10;
    } buttons;

    struct Axes {
      int forward = 1;
      int left = 0;
      int turn = 2;
      int throttle = 3;
      int fineAdjust = 4;
    } axes;

    struct Config {
      bool omniModeLatching = true;
    } config;
  };

  struct Result {
    DriveCommand command;
    bool requestEnable = false;
    bool requestDisable = false;
    bool servoChanged = false;
    double servoPos = 0.0;
  };

  JoystickInputHandler(const JoystickMap &joyMap, double vMax, double omegaMax);

  Result process(const sensor_msgs::msg::Joy::SharedPtr &joy, double currentServoPos);

private:
  const JoystickMap _joyMap;
  const double _vMax;
  const double _omegaMax;

  int _joyOmniPrev = -1;
  int _joyDisablePrev = -1;
  int _joyEnablePrev = -1;
  bool _omniModeEnabled = false;
};

} // namespace edu
