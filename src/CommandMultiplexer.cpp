#include "CommandMultiplexer.h"

#include <algorithm>
#include <cmath>

namespace edu {

bool DriveCommand::isNonZero(double epsilon) const {
  return std::abs(vFwd) > epsilon || std::abs(vLeft) > epsilon ||
         std::abs(omega) > epsilon;
}

CommandMultiplexer::CommandMultiplexer(double staleSeconds,
                                       double motionEpsilon)
    : _staleSeconds(staleSeconds), _motionEpsilon(motionEpsilon) {}

void CommandMultiplexer::updateJoystick(const DriveCommand &command,
                                        const rclcpp::Time &stamp) {
  _joystick = StampedCommand{command, stamp};
}

void CommandMultiplexer::updateVelocity(const DriveCommand &command,
                                        const rclcpp::Time &stamp) {
  _velocity = StampedCommand{command, stamp};
}

CommandMultiplexer::Selection
CommandMultiplexer::select(const rclcpp::Time &now) const {
  const bool joystickFresh = isFresh(_joystick, now);
  const bool velocityFresh = isFresh(_velocity, now);

  if (joystickFresh && _joystick->command.isNonZero(_motionEpsilon)) {
    return Selection{_joystick->command, InputSource::Joystick};
  }

  if (velocityFresh) {
    return Selection{_velocity->command, InputSource::Velocity};
  }

  return Selection{};
}

bool CommandMultiplexer::isFresh(const std::optional<StampedCommand> &input,
                                 const rclcpp::Time &now) const {
  if (!input.has_value()) {
    return false;
  }

  const rclcpp::Duration age = now - input->stamp;
  return age.seconds() <= _staleSeconds;
}

} // namespace edu
