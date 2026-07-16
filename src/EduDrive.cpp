#include "EduDrive.h"

#include <cmath>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdexcept>
#include <sys/ioctl.h>

#include "tf2/LinearMath/Quaternion.h"

namespace edu
{

EduDrive::EduDrive()
  : Node("edu_drive_node")
  , _servoPos(0.0)
{
}

EduDrive::~EduDrive()
{
}

void EduDrive::initDrive(
  std::vector<ControllerParams> cp, std::shared_ptr<SocketCAN> can, const JoystickInputHandler::JoystickMap& joyMap,
  bool using_pwr_mgmt, bool verbosity)
{
  _can            = can;
  _using_pwr_mgmt = using_pwr_mgmt;
  _verbosity      = verbosity;
  _enabled        = false;

  // CAN devices
  _adapter   = std::make_unique<RPiAdapterBoard>(can.get(), verbosity);
  _extension = std::make_unique<RPiExtensionBoard>(can.get(), verbosity);
  _pwr_mgmt  = std::make_unique<PowerManagementBoard>(can.get(), verbosity);

  _vMax     = 0.0;
  _omegaMax = 0.0;

  bool isKinematicsValid = true;

  for (const auto& controllerParams : cp)
  {
    for (const auto& motorParam : controllerParams.motorParams)
    {
      isKinematicsValid &= (motorParam.kinematics.size() == KINEMATIC_VECTOR_SIZE);
    }
  }

  if (!isKinematicsValid)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of length 3 are expected.");
    throw std::invalid_argument("Invalid kinematic vectors: expected vectors of length 3.");
  }

  std::vector<std::vector<double>> kinematicModel;
  for (unsigned int i = 0; i < cp.size(); ++i)
  {
    _mc.push_back(std::make_unique<edu::MotorController>(can.get(), cp[i], verbosity));

    for (unsigned int j = 0; j < _mc[i]->getMotorParams().size(); j++)
    {
      std::vector<double> kinematics = _mc[i]->getMotorParams()[j].kinematics;
      kinematicModel.push_back(kinematics);
      double kx     = kinematics[0];
      double kw     = kinematics[2];
      double rpmMax = cp[i].motorParams[0].rpmMax;
      for (unsigned int k = 1; k < MOTOR_CHANNELS; ++k)
        rpmMax = std::min(
          rpmMax,
          cp[i].motorParams[k].rpmMax); // the slowest motor determines the maximum speed of the system
      if (std::fabs(kx) > 1e-3)
      {
        double vMax = std::fabs(rpmMax * RPM2RADS / kx);
        if (vMax > _vMax)
          _vMax = vMax;
      }
      if (std::fabs(kw) > 1e-3)
      {
        double omegaMax = std::fabs(rpmMax * RPM2RADS / kw);
        if (omegaMax > _omegaMax)
          _omegaMax = omegaMax;
      }
    }
  }

  edu::Matrix K(kinematicModel);
  edu::Matrix Kinv = K.pseudoInverse();
  _odometry        = std::make_unique<Odometry>(edu::OdometryMode::Absolute, Kinv);
  _joystickInput   = std::make_unique<JoystickInputHandler>(joyMap, _vMax, _omegaMax);

  // Publisher of motor shields
  _pubEnabled = this->create_publisher<std_msgs::msg::ByteMultiArray>("enabled", 1);
  _pubRPM     = this->create_publisher<std_msgs::msg::Float32MultiArray>("rpm", 1);

  // Publisher of carrier shield
  _pubTemp           = this->create_publisher<std_msgs::msg::Float32>("temperature", 1);
  _pubVoltageAdapter = this->create_publisher<std_msgs::msg::Float32>("voltageAdapter", 1);
  _pubImu            = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);

  // Publisher of power management shield
  _pubVoltagePwrMgmt = this->create_publisher<std_msgs::msg::Float32>("voltagePwrMgmt", 1);
  _pubCurrentPwrMgmt = this->create_publisher<std_msgs::msg::Float32>("currentPwrMgmt", 1);

  // Broadcaster for odometry data
  _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  _odomTimer = create_wall_timer(ODOM_PUBLISH_INTERVAL, std::bind(&EduDrive::odomTimerCallback, this));

  // Subscribers last after everything else is initialized, to avoid receiving messages before the robot is ready
  _subJoy = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&EduDrive::joyCallback, this, std::placeholders::_1));
  _subVel = this->create_subscription<geometry_msgs::msg::Twist>(
    "vel/teleop", 10, std::bind(&EduDrive::velocityCallback, this, std::placeholders::_1));
  _srvEnable = this->create_service<std_srvs::srv::SetBool>(
    "enable",
    std::bind(&EduDrive::enableCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  _srvResetOdometry = this->create_service<std_srvs::srv::SetBool>(
    "reset_odometry",
    std::bind(
      &EduDrive::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Instantiated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");
}

void EduDrive::run()
{
  _lastCmd = this->get_clock()->now();

  rclcpp::TimerBase::SharedPtr timerReceiveCAN
    = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&EduDrive::hardwareWorker, this));
  rclcpp::TimerBase::SharedPtr timerCheckLaggyConnection
    = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EduDrive::checkLaggyConnection, this));

  rclcpp::spin(shared_from_this());

  _can->clearObservers();
  for (auto& mc : _mc)
  {
    mc->stop();
    mc->disable();
  }

  rclcpp::shutdown();
}

void EduDrive::enable()
{
  RCLCPP_INFO(this->get_logger(), "Enabling robot");

  if (_using_pwr_mgmt)
  {
    // Let power management board set hardware enable
    // if the power management board is not used, the user needs to take care of this pin.
    // The adapter board is designed to treat this pin as an input pin
    _pwr_mgmt->enable();
  }

  for (auto& mc : _mc)
  {
    if (!mc->isInitialized())
      mc->reinit();
    mc->enable();
  }
}

void EduDrive::disable()
{
  RCLCPP_INFO(this->get_logger(), "Disabling robot");

  if (_using_pwr_mgmt)
  {
    // Let power management board reset hardware enable
    _pwr_mgmt->disable();
  }

  for (auto& mc : _mc)
    mc->disable();
}

void EduDrive::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  if (!_joystickInput)
    return;

  const auto now   = this->get_clock()->now();
  const auto input = _joystickInput->process(joy, _servoPos);

  if (input.servoChanged)
  {
    _servoPos = input.servoPos;
    double angles[8];
    angles[0] = _servoPos;
    angles[1] = _servoPos;
    // these values mean, to not change servo position (values > 270 are ignored)
    angles[2] = 275;
    angles[3] = 275;
    angles[4] = 275;
    angles[5] = 275;
    angles[6] = 275;
    angles[7] = 275;
    _extension->setServos(angles);
  }

  if (input.requestDisable)
  {
    disable();
  }
  else if (input.requestEnable)
  {
    enable();
  }

  _commandMultiplexer.updateJoystick(input.command, now);
  const auto selected = _commandMultiplexer.select(now);
  controlMotors(selected.command.vFwd, selected.command.vLeft, selected.command.omega);
}

void EduDrive::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  const auto now = this->get_clock()->now();

  DriveCommand command;
  command.vFwd  = cmd->linear.x;
  command.vLeft = cmd->linear.y;
  command.omega = cmd->angular.z;

  _commandMultiplexer.updateVelocity(command, now);
  const auto selected = _commandMultiplexer.select(now);
  controlMotors(selected.command.vFwd, selected.command.vLeft, selected.command.omega);
}

bool EduDrive::enableCallback(
  const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
{
  // suppress warning about unused variable header
  (void)header;

  if (request->data == true)
  {
    RCLCPP_INFO(this->get_logger(), "%s", "Enabling robot");
    enable();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "%s", "Disabling robot");
    disable();
  }
  response->success = true;
  return true;
}

void EduDrive::odomTimerCallback(){
  if(!_odometry || !(_odometry->is_pos_init() || _odometry->is_vel_init()))
    return;

  Pose pose                    = _odometry->get_pose();

  geometry_msgs::msg::TransformStamped msgTransform;
  msgTransform.header.stamp    = this->get_clock()->now();
  msgTransform.header.frame_id = ODOM_FRAME_ID;
  msgTransform.child_frame_id  = BASE_FRAME_ID;

  tf2::Quaternion q_odom;
  q_odom.setEuler(0, 0, pose.theta);
  msgTransform.transform.translation.x = pose.x;
  msgTransform.transform.translation.y = pose.y;
  msgTransform.transform.translation.z = 0;
  msgTransform.transform.rotation.w    = q_odom.w();
  msgTransform.transform.rotation.x    = q_odom.x();
  msgTransform.transform.rotation.y    = q_odom.y();
  msgTransform.transform.rotation.z    = q_odom.z();
  _tf_broadcaster->sendTransform(msgTransform);
}

bool EduDrive::resetOdometryCallback(
  const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
{
  // suppress warning about unused variable header
  (void)header;

  if (!request->data)
  {
    response->success = false;
    response->message = "Set data=true to reset odometry";
    return true;
  }

  if (!_odometry)
  {
    response->success = false;
    response->message = "Odometry is not initialized";
    return true;
  }

  _odometry->reset();
  RCLCPP_INFO(this->get_logger(), "%s", "Odometry reset");

  response->success = true;
  response->message = "Odometry reset";
  return true;
}

void EduDrive::controlMotors(double vFwd, double vLeft, double omega)
{
  if (_mc.empty())
    return;

  _lastCmd = this->get_clock()->now();

  std::vector<std::array<double, MOTOR_CHANNELS>> motors(_mc.size());
  double scale = 1.0;

  // scale velocities relative to the slowest motor of the system
  for (unsigned int i = 0; i < _mc.size(); ++i)
  {
    for (unsigned int j = 0; j < MOTOR_CHANNELS; ++j)
    {

      const auto& rpmMax = _mc[i]->getMotorParams()[j].rpmMax;
      const auto& kin    = _mc[i]->getMotorParams()[j].kinematics;

      motors[i][j] = kin[0] * vFwd + kin[1] * vLeft + kin[2] * omega;
      motors[i][j] *= RADS2RPM;

      if (std::abs(motors[i][j]) > rpmMax)
      {
        scale = std::min(scale, rpmMax / std::abs(motors[i][j]));
      }
    }
  }

  // apply scale factor to all motors
  for (unsigned int i = 0; i < _mc.size(); ++i)
  {
    double w[MOTOR_CHANNELS];
    for (unsigned int j = 0; j < MOTOR_CHANNELS; ++j)
      w[j] = motors[i][j] * scale;
    _mc[i]->setRPM(w);

    if (_verbosity)
      RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1]);
  }
}

void EduDrive::hardwareWorker()
{
  const double voltageAdapter = _adapter->getVoltageSys();
  const double voltagePwrMgmt = _pwr_mgmt->getVoltage();

  std_msgs::msg::Float32MultiArray msgRPM;
  std_msgs::msg::ByteMultiArray msgEnabled;

  bool controllersInitialized = true;
  for (auto& mc : _mc)
  {
    controllersInitialized = controllersInitialized && mc->isInitialized();
  }

  for (auto& mc : _mc)
  {
    double response[MOTOR_CHANNELS] = {};
    bool enableState                = false;
    if (controllersInitialized)
    {
      if (voltageAdapter > 3.0 || voltagePwrMgmt > 3.0) //@ToDo: find nicer solution
      {
        if (mc->checkConnectionStatus(200))
        {
          mc->getWheelResponse(response);
          enableState = mc->getEnableState();
        }
        else
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "#EduDrive Error synchronizing with device" << mc->getCanId());
        }
      }
      else
      {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "#EduDrive Low voltage on drive power supply rail for device " << mc->getCanId());

        mc->deinit();
        disable();
      }
    }
    for (unsigned int j = 0; j < MOTOR_CHANNELS; ++j)
    {
      msgRPM.data.push_back(static_cast<float>(response[j]));
    }
    msgEnabled.data.push_back(enableState);
  }

  _enabled = false;
  if (msgEnabled.data.size() > 0)
  {
    _enabled = msgEnabled.data[0];
    for (unsigned int i = 1; i < msgEnabled.data.size(); i++)
    {
      _enabled &= msgEnabled.data[i];
    }
    _extension->sendEnabledState(_enabled);
  }

  rclcpp::Time stampReceived = this->get_clock()->now();
  _odometry->update(stampReceived.nanoseconds(), edu::Vec(msgRPM.data.begin(), msgRPM.data.end()));

  _pubRPM->publish(msgRPM);
  _pubEnabled->publish(msgEnabled);

  std_msgs::msg::Float32 msgTemperature;
  msgTemperature.data = static_cast<float>(_adapter->getTemperature());
  _pubTemp->publish(msgTemperature);

  std_msgs::msg::Float32 msgVoltageAdapter;
  msgVoltageAdapter.data = static_cast<float>(_adapter->getVoltageSys());
  _pubVoltageAdapter->publish(msgVoltageAdapter);

  double q[4], a[3], v[3];
  _adapter->getOrientation(q);
  _adapter->getAcceleration(a);
  _adapter->getAngularVelocity(v);
  sensor_msgs::msg::Imu msgImu;
  msgImu.header.stamp          = stampReceived;
  msgImu.header.frame_id       = BASE_FRAME_ID;
  msgImu.orientation.w         = q[0];
  msgImu.orientation.x         = q[1];
  msgImu.orientation.y         = q[2];
  msgImu.orientation.z         = q[3];
  msgImu.linear_acceleration.x = a[0];
  msgImu.linear_acceleration.y = a[1];
  msgImu.linear_acceleration.z = a[2];
  msgImu.angular_velocity.x    = v[0];
  msgImu.angular_velocity.y    = v[1];
  msgImu.angular_velocity.z    = v[2];
  _pubImu->publish(msgImu);

  std_msgs::msg::Float32 msgVoltagePwrMgmt;
  msgVoltagePwrMgmt.data = static_cast<float>(voltagePwrMgmt);
  _pubVoltagePwrMgmt->publish(msgVoltagePwrMgmt);

  std_msgs::msg::Float32 msgCurrentPwrMgmt;
  msgCurrentPwrMgmt.data = static_cast<float>(_pwr_mgmt->getCurrent());
  _pubCurrentPwrMgmt->publish(msgCurrentPwrMgmt);
}

void EduDrive::checkLaggyConnection()
{
  rclcpp::Duration dt = this->get_clock()->now() - _lastCmd;
  bool lag            = (dt.seconds() > 0.5);
  if (lag && _enabled)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Lag detected ... deactivate motor control");
    disable();
  }
}

int EduDrive::gpio_write(const char* dev_name, int offset, int value)
{
  struct gpiohandle_request rq;
  struct gpiohandle_data data;
  int fd, ret;

  fd = open(dev_name, O_RDONLY);
  if (fd < 0)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to open " << dev_name << ": " << strerror(errno) << std::endl);
    return -1;
  }
  rq.lineoffsets[0] = offset;
  rq.flags          = GPIOHANDLE_REQUEST_OUTPUT;
  rq.lines          = 1;
  ret               = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
  close(fd);
  if (ret == -1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to line handle from ioctl: " << strerror(errno) << std::endl);
    return -1;
  }
  data.values[0] = value;
  ret            = ioctl(rq.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
  if (ret == -1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to set line value using ioctl: " << strerror(errno) << std::endl);
    return -1;
  }
  else
  {
    if (_verbosity)
      std::cout << "Wrote value " << value << " to GPIO at offset " << offset << " (OUTPUT mode) on chip " << dev_name
                << std::endl;
  }

  close(rq.fd);
  return 1;
}

int EduDrive::gpio_read(const char* dev_name, int offset, int& value)
{
  struct gpiohandle_request rq;
  struct gpiohandle_data data;
  int fd, ret;
  fd = open(dev_name, O_RDONLY);
  if (fd < 0)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to open " << dev_name << ", " << strerror(errno) << std::endl);
    return -1;
  }
  rq.lineoffsets[0] = offset;
  rq.flags          = GPIOHANDLE_REQUEST_INPUT;
  rq.lines          = 1;
  ret               = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
  close(fd);
  if (ret == -1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to get line handle from ioctl : " << strerror(errno) << std::endl);
    return -1;
  }
  ret = ioctl(rq.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
  if (ret == -1)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unable to get line value using ioctl : " << strerror(errno) << std::endl);
    return -1;
  }
  else
  {
    if (_verbosity)
      std::cout << "Value of GPIO at offset " << offset << " (INPUT mode) on chip " << dev_name << ": "
                << data.values[0] << std::endl;
  }

  close(rq.fd);
  value = data.values[0];
  return 1;
}

} // namespace edu