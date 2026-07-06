#include "JoystickInputHandler.h"

#include <iostream>

namespace edu {

JoystickInputHandler::JoystickInputHandler(const JoystickMap &joyMap, double vMax, double omegaMax)
    : _joyMap(joyMap), _vMax(vMax), _omegaMax(omegaMax) {

    //if(verbosity)
    //{
    std::cout << "---------------------------" << std::endl << std::endl;
    std::cout << "--- Joystick Mapping ---" << std::endl;
    std::cout << "   --- Buttons ---" << std::endl;
    std::cout << "         enable     = " << joyMap.buttons.enable << std::endl;
    std::cout << "         disable    = " << joyMap.buttons.disable << std::endl;
    std::cout << "         omniMode   = " << joyMap.buttons.omniMode << std::endl;
    std::cout << "         servoLeft  = " << joyMap.buttons.servoLeft << std::endl;
    std::cout << "         servoRight = " << joyMap.buttons.servoRight << std::endl;
    std::cout << "   --- Axes ---" << std::endl;
    std::cout << "         forward    = " << joyMap.axes.forward << std::endl;
    std::cout << "         left       = " << joyMap.axes.left << std::endl;
    std::cout << "         turn       = " << joyMap.axes.turn << std::endl;
    std::cout << "         throttle   = " << joyMap.axes.throttle << std::endl;
    std::cout << "         fineAdjust = " << joyMap.axes.fineAdjust << std::endl;
    std::cout << "   --- Config ---" << std::endl;
    std::cout << "         omniModeLatching = " << joyMap.config.omniModeLatching << std::endl;
    std::cout << "---------------------------" << std::endl << std::endl;
    //}

  }

JoystickInputHandler::Result
JoystickInputHandler::process(const sensor_msgs::msg::Joy::SharedPtr &joy, double currentServoPos) {
  auto axis = [&](size_t idx, double fallback = 0.0) -> double {
    return (idx < joy->axes.size()) ? joy->axes[idx] : fallback;
  };

  auto button = [&](int idx) -> int32_t {
    if (idx < 0) {
      return 0;
    }

    const auto buttonIdx = static_cast<size_t>(idx);
    return (buttonIdx < joy->buttons.size()) ? joy->buttons[buttonIdx] : 0;
  };

  Result result;
  result.servoPos = currentServoPos;

  double fwd = axis(_joyMap.axes.forward);
  double left = axis(_joyMap.axes.left);
  double turn = axis(_joyMap.axes.turn);
  double throttle = (axis(_joyMap.axes.throttle, -1.0) + 1.0) / 2.0;

  const bool omniPressed = button(_joyMap.buttons.omniMode);
  if (_joyMap.config.omniModeLatching) {
    if (_joyOmniPrev < 0) {
      _omniModeEnabled = omniPressed;
    } else if (omniPressed && !_joyOmniPrev) {
      _omniModeEnabled = !_omniModeEnabled;
    }

    _joyOmniPrev = omniPressed;
  } else {
    _omniModeEnabled = omniPressed;
  }

  if (!_omniModeEnabled) {
    left = 0.0;
  }

  if (button(_joyMap.buttons.servoLeft)) {
    result.servoPos = 45.0;
  } else if (button(_joyMap.buttons.servoRight)) {
    result.servoPos = 225.0;
  }

  if (axis(_joyMap.axes.fineAdjust) == 1.0) {
    result.servoPos += 1.0;
  }
  if (axis(_joyMap.axes.fineAdjust) == -1.0) {
    result.servoPos -= 1.0;
  }

  if (result.servoPos < 0.0) {
    result.servoPos = 0.0;
  }
  if (result.servoPos > 270.0) {
    result.servoPos = 270.0;
  }

  result.servoChanged = (result.servoPos != currentServoPos);

  if (_joyDisablePrev < 0) {
    _joyDisablePrev = button(_joyMap.buttons.disable);
  }
  if (_joyEnablePrev < 0) {
    _joyEnablePrev = button(_joyMap.buttons.enable);
  }

  result.requestDisable = button(_joyMap.buttons.disable) && !_joyDisablePrev;
  result.requestEnable = button(_joyMap.buttons.enable) && !_joyEnablePrev;

  _joyDisablePrev = button(_joyMap.buttons.disable);
  _joyEnablePrev = button(_joyMap.buttons.enable);

  result.command.vFwd = throttle * fwd * _vMax;
  result.command.vLeft = throttle * left * _vMax;
  result.command.omega = throttle * turn * _omegaMax;

  return result;
}

} // namespace edu
