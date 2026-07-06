#pragma once

#include "MotorController.h"
#include "Odometry.h"
#include "PowerManagementBoard.h"
#include "RPiAdapterBoard.h"
#include "RPiExtensionBoard.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <vector>

namespace edu {

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

/**
 * @class EduDrive
 * @brief Drive interface to EduArt's stackable motor controllers
 * @author Stefan May
 * @date 27.04.2022
 */
class EduDrive : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   *
   */
  EduDrive();

  /**
   * @brief Destructor
   *
   */
  ~EduDrive();

  /**
   * @brief Initialize drive
   * @param[in] cp Parameters for the closed-loop controller of all motor
   * channels
   * @param[in] can Instance of CAN communication socket. All connected devices
   * share the communication bus.
   * @param[in] using_pwr_mgmt Flag indicating whether the power management
   * module of EduArt is used. This enables additional measurement topics
   * (voltage and current sensing).
   * @param[in] verbosity Make the instance of this class more chatty. Debug
   * information will be printed to stdout.
   */
  void initDrive(std::vector<ControllerParams> cp,
                 std::shared_ptr<SocketCAN> can, const JoystickMap &joystickMap,
                 bool using_pwr_mgmt = true, bool verbosity = false);

  /**
   * @brief Blocking ROS handler method. Call this method to enter the ROS
   * message loop.
   */
  void run();

  /**
   * @brief Enable all drives
   */
  void enable();

  /**
   * @brief Disable all drives
   */
  void disable();

  /**
   * @brief Method called by ROS as joystick data is available
   * @param joy joystick data
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

  /**
   * @brief Method called by ROS as twist data is available
   */
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);

private:
  static constexpr double RADS2RPM = 60.0 / (2.0 * M_PI);
  static constexpr double RPM2RADS = (2.0 * M_PI) / 60.0;
  static constexpr unsigned int MOTOR_CHANNELS = 2;
  static constexpr unsigned int KINEMATIC_VECTOR_SIZE = 3;

  void hardwareWorker();

  void checkLaggyConnection();

  int gpio_write(const char *dev_name, int offset, int value);

  int gpio_read(const char *dev_name, int offset, int &value);

  void controlMotors(double vFwd, double vLeft, double omega);

  bool enableCallback(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool_Response> response);

  // Input topics / services
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subJoy;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subVel;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _srvEnable;

  // Data available from motor controllers
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr _pubEnabled;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _pubRPM;

  // Data available from adapter board
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pubTemp;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pubVoltageAdapter;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _pubImu;

  // Data available from power management board
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pubVoltagePwrMgmt;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pubCurrentPwrMgmt;

  // Odometry
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  rclcpp::Time _lastCmd;           // Time elapsed since last call
  std::shared_ptr<SocketCAN> _can; // Pointer to CAN instance

  std::unique_ptr<Odometry> _odometry;             // Odometry model of robot
  std::unique_ptr<RPiAdapterBoard> _adapter;       // Adapter board
  std::unique_ptr<RPiExtensionBoard> _extension;   // Extension board
  std::unique_ptr<PowerManagementBoard> _pwr_mgmt; // Power management board
  std::vector<std::unique_ptr<MotorController>>
      _mc; // Vector containing pointer to all motor controller instances

  double _vMax;
  double _omegaMax;
  double _servoPos;
  JoystickMap _joyMap;
  int _joyOmniPrev = -1;
  int _joyDisablePrev = -1;
  int _joyEnablePrev = -1;
  bool _omniModeEnabled = false;

  bool _enabled;
  bool _using_pwr_mgmt;
  bool _verbosity;
};

} // namespace edu