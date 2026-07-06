#include "MotorController.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <cstring>
#include <iomanip>
#include <unistd.h>
#include "can/canprotocol.h"
#include "rclcpp/rclcpp.hpp"

namespace edu
{

MotorController::MotorController(SocketCAN* can, ControllerParams params, bool verbosity)
  : _params(params),
    _verbosity(verbosity)
{
  _isInit    = false;
  
  //if(verbosity)
  //{
  std::cout << "---------------------------" << std::endl << std::endl;
  std::cout << "frequencyScale = " << params.frequencyScale << std::endl;
  std::cout << "inputWeight    = " << params.inputWeight << std::endl;
  std::cout << "maxPulseWidth  = " << (int)params.maxPulseWidth << std::endl;
  std::cout << "timeout        = " << params.timeout << std::endl;

  std::cout << "kp             = " << params.kp << std::endl;
  std::cout << "ki             = " << params.ki << std::endl;
  std::cout << "kd             = " << params.kd << std::endl;
  std::cout << "antiWindup     = " << params.antiWindup << std::endl;
  std::cout << "responseMode   = " << static_cast<int>(params.responseMode) << std::endl;

  std::cout << std::endl << "--- Controller #" << std::hex << params.canID << std::dec << " parameters ---" << std::endl;

  for(unsigned int i=0; i<2; i++)
  {
    std::cout << "   --- Drive" << i << " ---" << std::endl;
    std::cout << "         channel: " << _params.motorParams[i].channel << std::endl;
    std::cout << "         kinematics: ";
    for(unsigned int j=0; j<_params.motorParams[i].kinematics.size(); j++)
      std::cout << _params.motorParams[i].kinematics[j] << " ";
    std::cout << std::endl;
    std::cout << "         gearRatio      = " << params.motorParams[i].gearRatio << std::endl;
    std::cout << "         encoderRatio   = " << params.motorParams[i].encoderRatio << std::endl;
    std::cout << "         rpmMax         = " << params.motorParams[i].rpmMax << std::endl;
    std::cout << "         invertEnc      = " << params.motorParams[i].invertEnc << std::endl;
  }

  std::cout << "---------------------------" << std::endl << std::endl;
  //}

  _can       = can;
  makeCanStdID(SYSID_MC2, params.canID, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MotorController"), "#MotorController CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl);

  canid_t canidOutput = _outputAddress;

  setCANId(canidOutput);
  can->registerObserver(this);

  init();
}

MotorController::~MotorController()
{

}

bool MotorController::isInitialized()
{
  LockGuard guard(_stateMutex);
  return _isInit;
}

void MotorController::init()
{
  if(!requestFirmwareVersion(100ms))
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MotorController"), "#MotorController legacy firmware detected on controller " << _params.canID << ". Can't set individual motor parameters per channel." << std::endl);
  }

  _rpm[0] = 0.f;
  _rpm[1] = 0.f;

  _enabled       = false;

  bool retval = true;
  
  if(!disable())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Failed to disable device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setFrequencyScale(_params.frequencyScale))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting frequency scaling parameter failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setInputWeight(_params.inputWeight))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting differential factor of PID controller failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setMaxPulseWidth(_params.maxPulseWidth))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting maximum pulse width failed for device " << _params.canID << std::endl);
    retval = false;
  }

  if(!setTimeout(_params.timeout))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting timeout failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  double gearRatios[] = {_params.motorParams[0].gearRatio, _params.motorParams[1].gearRatio};
  if(!setGearRatio(gearRatios))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting gear ratio failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  double ticksPerRev[] = {_params.motorParams[0].encoderRatio, _params.motorParams[1].encoderRatio};
  if(!setEncoderTicksPerRev(ticksPerRev))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting encoder parameters failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setKp(_params.kp))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting proportional factor of PID controller failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setKi(_params.ki))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting integration factor of PID controller failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setKd(_params.kd))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting differential factor of PID controller failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!setAntiWindup(_params.antiWindup))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting anti-windup factor of PID controller failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  if(!configureResponse(_params.responseMode))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting response mode failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  bool invertEnc[] = {(bool)_params.motorParams[0].invertEnc, (bool)_params.motorParams[1].invertEnc};
  if(!invertEncoderPolarity(invertEnc))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Setting encoder polarity failed for device " << _params.canID << std::endl);
    retval = false;
  }
  
  usleep(25000);

  if(retval)
  {
    LockGuard guard(_stateMutex);
    _isInit = true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "#MotorController ERROR initializing motor controller with ID " << _params.canID << std::endl);
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorController"), "-----------------------------------------------");
  }
}

void MotorController::deinit()
{
  LockGuard guard(_stateMutex);
  _isInit = false;
}

void MotorController::reinit()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("MotorController"), "#MotorController Reinitializing device " << _params.canID << std::endl);
  init();
}

bool MotorController::enable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_ENABLE;
  return _can->send(&_cf);
}

bool MotorController::disable()
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_DISABLE;
  return _can->send(&_cf);
}

bool MotorController::getEnableState()
{
  LockGuard guard(_stateMutex);
  return _enabled;
}

bool MotorController::broadcastExternalSync()
{
  canid_t idTmp = _cf.can_id;
  _cf.can_id = _broadcastAddress;
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_SYNC;
  bool retval = _can->send(&_cf);
  _cf.can_id = idTmp;
  return retval;
}

const std::vector<MotorParams>& MotorController::getMotorParams()
{
  return _params.motorParams;
}

bool MotorController::configureResponse(CanResponseMode mode)
{
  _cf.can_dlc = 1;
  if(mode==CanResponseMode::Rpm)
    _cf.data[0] = CMD_MOTOR_SENDRPM;
  else
    _cf.data[0] = CMD_MOTOR_SENDPOS;
  return _can->send(&_cf);
}

bool MotorController::invertEncoderPolarity(bool invert[2])
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_MOTOR_INVERTENC;
  _cf.data[1] = (invert[0]) ? 1 : 0;
  _cf.data[2] = (invert[1]) ? 1 : 0;
  return _can->send(&_cf);
}

unsigned short MotorController::getCanId()
{
  return _cf.can_id & 0xF;
}

bool MotorController::setTimeout(unsigned short timeoutInMillis)
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_MOTOR_SETTIMEOUT;
  _cf.data[1] = (timeoutInMillis >> 8) & 0xFF;
  _cf.data[2] = timeoutInMillis & 0xFF;
  bool retval = _can->send(&_cf);
  if(retval)
    _params.timeout = timeoutInMillis;
  return retval;
}

unsigned short MotorController::getTimeout()
{
  return _params.timeout;
}

bool MotorController::setGearRatio(double gearRatio[2])
{
  bool retval  = sendFloat(CMD_MOTOR_GEARRATIO, static_cast<float>(gearRatio[0]), 0);
  retval      &= sendFloat(CMD_MOTOR_GEARRATIO, static_cast<float>(gearRatio[1]), 1);

  if(retval){
    _params.motorParams[0].gearRatio = gearRatio[0];
    _params.motorParams[1].gearRatio = gearRatio[1];
  }
  return retval;
}

double MotorController::getGearRatio(size_t motor_num)
{
  return _params.motorParams[motor_num].gearRatio;
}

bool MotorController::setEncoderTicksPerRev(double encoderTicksPerRev[2])
{
  bool retval  = sendFloat(CMD_MOTOR_TICKSPERREV, static_cast<float>(encoderTicksPerRev[0]), 0);
  retval      &= sendFloat(CMD_MOTOR_TICKSPERREV, static_cast<float>(encoderTicksPerRev[1]), 1);
  if(retval){
    _params.motorParams[0].encoderRatio = encoderTicksPerRev[0];
    _params.motorParams[1].encoderRatio = encoderTicksPerRev[1];
  }
  return retval;
}

bool MotorController::setFrequencyScale(unsigned short scale)
{
  bool retval = false;

  if(scale>0 && scale<=100)
  {
    _cf.can_dlc = 3;
    _cf.data[0] = CMD_MOTOR_FREQ_SCALE;
    _cf.data[1] = (scale >> 8) & 0xFF;
    _cf.data[2] = scale & 0xFF;
    retval = _can->send(&_cf);
  }
  if(retval)
    _params.frequencyScale = scale;
  return retval;
}

unsigned short MotorController::getFrequencyScale()
{
  return _params.frequencyScale;
}

bool MotorController::setMaxPulseWidth(unsigned char pulse)
{
  bool retval = false;

  if(pulse<=127)
  {
    _cf.can_dlc = 2;
    _cf.data[0] = CMD_MOTOR_SETPWMMAX;
    _cf.data[1] = pulse;
    retval = _can->send(&_cf);
  }
  if(retval)
    _params.maxPulseWidth = pulse;
  return retval;
}

bool MotorController::setPWM(int pwm[2])
{
  _cf.can_dlc = 3;

  int vel1 = pwm[0];
  int vel2 = pwm[1];
  if(vel1>100)  vel1 = 100;
  if(vel1<-100) vel1 = -100;
  if(vel2>100)  vel2 = 100;
  if(vel2<-100) vel2 = -100;

  _cf.data[0] = CMD_MOTOR_SETPWM;
  _cf.data[1] = (char)vel1;
  _cf.data[2] = (char)vel2;

  return _can->send(&_cf);
}

bool MotorController::setRPM(double rpm[2])
{
  _cf.can_dlc = 5;

  int vel1 = (int)(rpm[0]*100.0);
  int vel2 = (int)(rpm[1]*100.0);

  _cf.data[0] = CMD_MOTOR_SETRPM;
  _cf.data[1] = (char)(vel1 >> 8) & 0xFF;
  _cf.data[2] = (char)(vel1)      & 0xFF;
  _cf.data[3] = (char)(vel2 >> 8) & 0xFF;
  _cf.data[4] = (char)(vel2)      & 0xFF;

  return _can->send(&_cf);
}

void MotorController::getWheelResponse(double response[2])
{
  LockGuard guard(_stateMutex);
  if(_params.responseMode == CanResponseMode::Rpm)
  {
    response[0] = _rpm[0];
    response[1] = _rpm[1];
  }
  else
  {
    response[0] = _pos[0];
    response[1] = _pos[1];
  }
}

bool MotorController::setKp(double kp)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KP, static_cast<float>(kp));
  if(retval)
    _params.kp = kp;
  return retval;
}

double MotorController::getKp()
{
  return _params.kp;
}

bool MotorController::setKi(double ki)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KI, static_cast<float>(ki));
  if(retval)
    _params.ki = ki;
  return retval;
}

double MotorController::getKi()
{
  return _params.ki;
}

bool MotorController::setKd(double kd)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KD, static_cast<float>(kd));
  if(retval)
    _params.kd = kd;
  return retval;
}

double MotorController::getKd()
{
  return _params.kd;
}

bool MotorController::setAntiWindup(bool activate)
{
   bool retval = false;
  _cf.can_dlc = 2;
  _cf.data[0] = CMD_MOTOR_CTL_ANTIWINDUP;
  _cf.data[1] = activate;
  retval = _can->send(&_cf);
  if(retval)
    _params.antiWindup = activate;
  return retval;
}

bool MotorController::getAntiWindup()
{
  return _params.antiWindup;
}

bool MotorController::setInputWeight(double weight)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_INPUTFILTER, static_cast<float>(weight));
  if(retval)
    _params.inputWeight = weight;
  return retval;
}

double MotorController::getInputWeight()
{
  return _params.inputWeight;
}

Version MotorController::getFirmwareVersion()
{
  LockGuard guard(_stateMutex);
  return _version;
}

bool MotorController::requestFirmwareVersion(std::chrono::milliseconds timeout)
{
  _cf.can_dlc = 1;
  _cf.data[0] = CMD_MOTOR_GET_FIRMWARE;
  _can->send(&_cf);

  auto startTime = std::chrono::steady_clock::now();

  if(timeout > 0ms){
    while(((std::chrono::steady_clock::now() - startTime) < timeout))
    {
      {
        LockGuard guard(_stateMutex);
        if(_version.isValid()) break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  {
    LockGuard guard(_stateMutex);
    return _version.isValid();
  }
}

void MotorController::notify(struct can_frame* frame)
{
  LockGuard guard(_stateMutex);
  if(frame->can_dlc==6)
  {
    if(frame->data[0] == RESPONSE_MOTOR_RPM)
    {
    
      short val1 = (frame->data[1] << 8 | (frame->data[2]));
      short val2 = (frame->data[3] << 8 | (frame->data[4]));
      _rpm[0] = static_cast<double>(val1) / 100.0;
      _rpm[1] = static_cast<double>(val2) / 100.0;
      _pos[0] = 0.f;
      _pos[1] = 0.f;
    }
    else if(frame->data[0] == RESPONSE_MOTOR_POS)
    {
      _rpm[0] = 0.f;
      _rpm[1] = 0.f;
      _pos[0] = (frame->data[1] | (frame->data[2] << 8));
      _pos[1] = (frame->data[3] | (frame->data[4] << 8));
    }
    _enabled = (frame->data[5] != 0);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MotorController"), "MotorController CANID " << _cf.can_id << " received data" << std::endl);
  }
  else if(frame->can_dlc==8){
    if(frame->data[0] == RESPONSE_MOTOR_PARAMETER && frame->data[1] == CMD_MOTOR_GET_FIRMWARE)
    {
      _version.major = (frame->data[3] | (frame->data[2] << 8));
      _version.minor = (frame->data[5] | (frame->data[4] << 8));
      _version.patch = (frame->data[7] | (frame->data[6] << 8));

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MotorController"), "MotorController has firmware version " << (int)_version.major << "." << (int)_version.minor << "." << (int)_version.patch << std::endl);
    }
  }
}

void MotorController::stop()
{
  _cf.can_dlc = 3;
  _cf.data[0] = CMD_MOTOR_SETPWM;
  _cf.data[1] = 0x0;
  _cf.data[2] = 0x0;
  _can->send(&_cf);
}

bool MotorController::sendFloat(int cmd, float f)
{
  _cf.can_dlc = 5;

  _cf.data[0] = cmd;
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof(u));
  _cf.data[1] = (u & 0xFF000000) >> 24;
  _cf.data[2] = (u & 0x00FF0000) >> 16;
  _cf.data[3] = (u & 0x0000FF00) >> 8;
  _cf.data[4] = (u & 0x000000FF);

  return _can->send(&_cf);
}

bool MotorController::sendFloat(int cmd, float f, int channel)
{
  _cf.can_dlc = 6;

  _cf.data[0] = cmd;
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof(u));
  _cf.data[1] = (u & 0xFF000000) >> 24;
  _cf.data[2] = (u & 0x00FF0000) >> 16;
  _cf.data[3] = (u & 0x0000FF00) >> 8;
  _cf.data[4] = (u & 0x000000FF);
  _cf.data[5] = channel;

  return _can->send(&_cf);
}

}
