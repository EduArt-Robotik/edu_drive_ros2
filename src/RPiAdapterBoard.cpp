#include "RPiAdapterBoard.h"
#include <iostream>
#include "can/canprotocol.h"
#include <unistd.h>

namespace edu
{

RPiAdapterBoard::RPiAdapterBoard(SocketCAN* can, bool verbosity)
{
  _init = false;
  _verbosity = verbosity;
  
  _q[0] = 1.0;
  _q[1] = 0.0;
  _q[2] = 0.0;
  _q[3] = 0.0;

  _temperature  = -273.0;
  _voltageSys   = 0.0;

  _acceleration[0] = 0.0;
  _acceleration[1] = 0.0;
  _acceleration[2] = 0.0;

  _angular_velocity[0] = 0.0;
  _angular_velocity[1] = 0.0;
  _angular_velocity[2] = 0.0;

  makeCanStdID(SYSID_RPI_ADAPTER, RPI_ADAPTER, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  if(verbosity)
    std::cout << "#CarrierBoard CAN Input ID: " << std::hex << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

  canid_t canidOutput = _outputAddress;

  setCANId(canidOutput);
  can->registerObserver(this);
  
  for(unsigned int i=0; i<500; i++)
  {
    if(_init) break;
    usleep(10000);
  }
}

RPiAdapterBoard::~RPiAdapterBoard()
{

}

void RPiAdapterBoard::getOrientation(double q[4])
{
  q[0] = _q[0];
  q[1] = _q[1];
  q[2] = _q[2];
  q[3] = _q[3];
}

double RPiAdapterBoard::getTemperature()
{
  return _temperature;
}

double RPiAdapterBoard::getVoltageSys()
{
  return _voltageSys;
}

void RPiAdapterBoard::getAcceleration(double acc[3])
{
  acc[0] = _acceleration[0];
  acc[1] = _acceleration[1];
  acc[2] = _acceleration[2];
}

void RPiAdapterBoard::getAngularVelocity(double vel[3])
{
  vel[0] = _angular_velocity[0];
  vel[1] = _angular_velocity[1];
  vel[2] = _angular_velocity[2];
}

void RPiAdapterBoard::notify(struct can_frame* frame)
{
  uint8_t* data = frame->data;
  
  if(frame->can_dlc==8) // Receive quaternion
  {
    int16_t iFused[4];
    iFused[0] = ((data[0] << 8) & 0xFF00) | data[1];
    iFused[1] = ((data[2] << 8) & 0xFF00) | data[3];
    iFused[2] = ((data[4] << 8) & 0xFF00) | data[5];
    iFused[3] = ((data[6] << 8) & 0xFF00) | data[7];
    _q[0] = ((double)iFused[0]) / 10000.0;
    _q[1] = ((double)iFused[1]) / 10000.0;
    _q[2] = ((double)iFused[2]) / 10000.0;
    _q[3] = ((double)iFused[3]) / 10000.0;
    
    if(_verbosity)
      std::cout << "w=" << _q[0] << " x=" << _q[1] << " y=" << _q[2] << " z=" << _q[3] << std::endl;
  }
  else if(frame->can_dlc==4) // Receive temperature and voltage
  {
    int16_t temp    = ((data[0] << 8) & 0xFF00) | data[1];
    int16_t voltage = ((data[2] << 8) & 0xFF00) | data[3];
    _temperature    = ((double)temp) / 100.0;
    _voltageSys     = ((double)voltage) / 100.0;
    
    if(_verbosity)
      std::cout << "T=" << _temperature << "°C Vsys=" << _voltageSys << std::endl;
      
    _init = true;
  }
  else if(frame->can_dlc==7) // Receive IMU measurements
  {
    // linear acceleration
    if(frame->data[0] == 1)
    {
      int16_t acc[3];
      acc[0] = ((data[1] << 8) & 0xFF00) | data[2];
      acc[1] = ((data[3] << 8) & 0xFF00) | data[4];
      acc[2] = ((data[5] << 8) & 0xFF00) | data[6];
      _acceleration[0] = ((double)acc[0]) / 1000.0;
      _acceleration[1] = ((double)acc[1]) / 1000.0;
      _acceleration[2] = ((double)acc[2]) / 1000.0;
    }
    //angular velocity
    else if(frame->data[0] == 2)
    {
      int16_t vel[3];
      vel[0] = ((data[1] << 8) & 0xFF00) | data[2];
      vel[1] = ((data[3] << 8) & 0xFF00) | data[4];
      vel[2] = ((data[5] << 8) & 0xFF00) | data[6];
      _angular_velocity[0] = ((double)vel[0]) / 1000.0;
      _angular_velocity[1] = ((double)vel[1]) / 1000.0;
      _angular_velocity[2] = ((double)vel[2]) / 1000.0;
    }
    else
    {
      std::cout << "RPiAdapterBoard::notify: Warning - wrong protocol in acceleration message." << std::endl;
    }

  }
  else
  {
    std::cout << "RPiAdapterBoard::notify: Warning - wrong message format received." << std::endl;
  }
}

}
