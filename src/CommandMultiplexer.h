#pragma once

#include "rclcpp/time.hpp"

#include <optional>

namespace edu {

enum class InputSource { None, Joystick, Velocity };

struct DriveCommand {
  double vFwd = 0.0;
  double vLeft = 0.0;
  double omega = 0.0;

  bool isNonZero(double epsilon) const;
};

class CommandMultiplexer {
public:
  struct Selection {
    DriveCommand command;
    InputSource source = InputSource::None;
  };

  explicit CommandMultiplexer(double staleSeconds = 0.5,
                              double motionEpsilon = 1e-4);

  void updateJoystick(const DriveCommand &command, const rclcpp::Time &stamp);
  void updateVelocity(const DriveCommand &command, const rclcpp::Time &stamp);

  Selection select(const rclcpp::Time &now) const;

private:
  struct StampedCommand {
    DriveCommand command;
    rclcpp::Time stamp;
  };

  bool isFresh(const std::optional<StampedCommand> &input,
               const rclcpp::Time &now) const;

  double _staleSeconds;
  double _motionEpsilon;
  std::optional<StampedCommand> _joystick;
  std::optional<StampedCommand> _velocity;
};

} // namespace edu
