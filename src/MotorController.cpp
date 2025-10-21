#include "MotorController.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>
#include "can/canprotocol.h"

namespace edu
{

MotorController::MotorController(SocketCAN* can, ControllerParams params, bool verbosity) : _params(params), _verbosity(verbosity)
{
  _isInit    = false;
  
  if(true)
  {
    std::cout << "---------------------------" << std::endl << std::endl;
    std::cout << "frequencyScale = " << params.frequencyScale << std::endl;
    std::cout << "inputWeight    = " << params.inputWeight << std::endl;
    std::cout << "maxPulseWidth  = " << (int)params.maxPulseWidth << std::endl;
    std::cout << "timeout        = " << params.timeout << std::endl;

    std::cout << "kp             = " << params.kp << std::endl;
    std::cout << "ki             = " << params.ki << std::endl;
    std::cout << "kd             = " << params.kd << std::endl;
    std::cout << "antiWindup     = " << params.antiWindup << std::endl;
    std::cout << "responseMode   = " << params.responseMode << std::endl;

    std::cout << std::endl << "--- Controller #" << std::hex << params.canID << std::dec << " parameters ---" << std::endl;

    for(unsigned int i=0; i<2; i++)
    {
      std::cout << "   --- Drive" << i << std::endl;
      std::cout << "       channel: " << _params.motorParams[i].channel << std::endl;
      std::cout << "       kinematics: ";
      for(unsigned int j=0; j<_params.motorParams[i].kinematics.size(); j++)
        std::cout << _params.motorParams[i].kinematics[j] << " ";
      std::cout << std::endl;
      std::cout << "       gearRatio      = " << params.motorParams[i].gearRatio << std::endl;
      std::cout << "       encoderRatio   = " << params.motorParams[i].encoderRatio << std::endl;
      std::cout << "       rpmMax         = " << params.motorParams[i].rpmMax << std::endl;
      std::cout << "       invertEnc      = " << params.motorParams[i].invertEnc << std::endl;
    }

    std::cout << "---------------------------" << std::endl << std::endl;
  }

  _can       = can;
  makeCanStdID(SYSID_MC2, params.canID, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  if(verbosity)
    std::cout << "#MotorController CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

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
  return _isInit;
}

void MotorController::init()
{
  _rpm[0] = 0.f;
  _rpm[1] = 0.f;

  _enabled       = false;

  bool retval = true;
  
  if(!disable())
  {
    std::cout << "#MotorController Failed to disable device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setFrequencyScale(_params.frequencyScale))
  {
    std::cout << "#MotorController Setting frequency scaling parameter failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setInputWeight(_params.inputWeight))
  {
    std::cout << "#MotorController Setting differential factor of PID controller failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setMaxPulseWidth(_params.maxPulseWidth))
  {
    std::cout << "#MotorController Setting maximum pulse width failed for device " << _params.canID << std::endl;
    retval = false;
  }

  if(!setTimeout(_params.timeout))
  {
    std::cout << "#MotorController Setting timeout failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  float gearRatios[] = {_params.motorParams[0].gearRatio, _params.motorParams[1].gearRatio};
  if(!setGearRatio(gearRatios))
  {
    std::cout << "#MotorController Setting gear ratio failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  float ticksPerRev[] = {_params.motorParams[0].encoderRatio, _params.motorParams[1].encoderRatio};
  if(!setEncoderTicksPerRev(ticksPerRev))
  {
    std::cout << "#MotorController Setting encoder parameters failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setKp(_params.kp))
  {
    std::cout << "#MotorController Setting proportional factor of PID controller failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setKi(_params.ki))
  {
    std::cout << "#MotorController Setting integration factor of PID controller failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setKd(_params.kd))
  {
    std::cout << "#MotorController Setting differential factor of PID controller failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!setAntiWindup(_params.antiWindup))
  {
    std::cout << "#MotorController Setting differential factor of PID controller failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  if(!configureResponse(_params.responseMode))
  {
    std::cout << "#MotorController Setting response mode failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  bool invertEnc[] = {(bool)_params.motorParams[0].invertEnc, (bool)_params.motorParams[1].invertEnc};
  if(!invertEncoderPolarity(invertEnc))
  {
    std::cout << "#MotorController Setting encoder polarity failed for device " << _params.canID << std::endl;
    retval = false;
  }
  
  usleep(25000);

  if(retval)
  {
    _isInit = true;
  }
  else
  {
    std::cout << "#MotorController ERROR initializing motor controller with ID " << _params.canID << std::endl;
    std::cout << "-----------------------------------------------";
  }
}

void MotorController::deinit()
{
  _isInit = false;
}

void MotorController::reinit()
{
  std::cout << "#MotorController Reinitializing device " << _params.canID << std::endl;
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

std::vector<MotorParams> MotorController::getMotorParams()
{
  return _params.motorParams;
}

bool MotorController::configureResponse(enum CanResponse mode)
{
  _cf.can_dlc = 1;
  if(mode==CAN_RESPONSE_RPM)
    _cf.data[0] = CMD_MOTOR_SENDRPM;
  else
    _cf.data[0] = CMD_MOTOR_SENDPOS;
  return _can->send(&_cf);
}

bool MotorController::invertEncoderPolarity(bool invert[2])
{
  _cf.can_dlc = 2;
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

bool MotorController::setGearRatio(float gearRatio[2])
{
  bool retval  = sendFloat(CMD_MOTOR_GEARRATIO,  gearRatio[0]);
  retval      &= sendFloat(CMD_MOTOR_GEARRATIO2, gearRatio[1]);
  if(retval){
    _params.motorParams[0].gearRatio = gearRatio[0];
    _params.motorParams[1].gearRatio = gearRatio[1];
  }
  return retval;
}

float MotorController::getGearRatio(size_t motor_num)
{
  return _params.motorParams[motor_num].gearRatio;
}

bool MotorController::setEncoderTicksPerRev(float encoderTicksPerRev[2])
{
  bool retval  = sendFloat(CMD_MOTOR_TICKSPERREV,  encoderTicksPerRev[0]);
  retval      &= sendFloat(CMD_MOTOR_TICKSPERREV2, encoderTicksPerRev[1]);
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

bool MotorController::setRPM(float rpm[2])
{
  _cf.can_dlc = 5;

  int vel1 = (int)(rpm[0]*100.f);
  int vel2 = (int)(rpm[1]*100.f);

  _cf.data[0] = CMD_MOTOR_SETRPM;
  _cf.data[1] = (char)(vel1 >> 8) & 0xFF;
  _cf.data[2] = (char)(vel1)      & 0xFF;
  _cf.data[3] = (char)(vel2 >> 8) & 0xFF;
  _cf.data[4] = (char)(vel2)      & 0xFF;

  return _can->send(&_cf);
}

void MotorController::getWheelResponse(float response[2])
{
  if(_params.responseMode == CAN_RESPONSE_RPM)
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

bool MotorController::setKp(float kp)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KP, kp);
  if(retval)
    _params.kp = kp;
  return retval;
}

float MotorController::getKp()
{
  return _params.kp;
}

bool MotorController::setKi(float ki)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KI, ki);
  if(retval)
    _params.ki = ki;
  return retval;
}

float MotorController::getKi()
{
  return _params.ki;
}

bool MotorController::setKd(float kd)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_KD, kd);
  if(retval)
    _params.kd = kd;
  return retval;
}

float MotorController::getKd()
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

bool MotorController::setInputWeight(float weight)
{
  bool retval = sendFloat(CMD_MOTOR_CTL_INPUTFILTER, weight);
  if(retval)
    _params.inputWeight = weight;
  return retval;
}

float MotorController::getInputWeight()
{
  return _params.inputWeight;
}

void MotorController::notify(struct can_frame* frame)
{
  if(frame->can_dlc==6)
  {
    if(frame->data[0] == RESPONSE_MOTOR_RPM)
    {
    
      short val1 = (frame->data[1] << 8 | (frame->data[2]));
      short val2 = (frame->data[3] << 8 | (frame->data[4]));
      _rpm[0] = ((float)val1) / 100.f;
      _rpm[1] = ((float)val2) / 100.f;
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
    if(_verbosity)
	    std::cout << "MotorController CANID " << _cf.can_id << " received data" << std::endl;
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
  int* ival = (int*)&f;
  _cf.data[1] = (*ival & 0xFF000000) >> 24;
  _cf.data[2] = (*ival & 0x00FF0000) >> 16;
  _cf.data[3] = (*ival & 0x0000FF00) >> 8;
  _cf.data[4] = (*ival & 0x000000FF);

  return _can->send(&_cf);
}

}
