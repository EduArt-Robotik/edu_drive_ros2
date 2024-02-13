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

  _temperature  = -273.f;
  _voltageSys   = 0.f;

  makeCanStdID(SYSID_RPI_ADAPTER, RPI_ADAPTER, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  if(verbosity)
    std::cout << "#CarrierBoard CAN Input ID: " << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;

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

float RPiAdapterBoard::getTemperature()
{
  return _temperature;
}

float RPiAdapterBoard::getVoltageSys()
{
  return _voltageSys;
}

void RPiAdapterBoard::notify(struct can_frame* frame)
{
  if(frame->can_dlc==8)
  {
    int16_t* data = (int16_t*)frame->data;
    _q[0] = ((double)data[0]) / 10000.0;
    _q[1] = ((double)data[1]) / 10000.0;
    _q[2] = ((double)data[2]) / 10000.0;
    _q[3] = ((double)data[3]) / 10000.0;
    
    if(_verbosity)
      std::cout << "w=" << _q[0] << " x=" << _q[1] << " y=" << _q[2] << " z=" << _q[3] << std::endl;
  }
  else if(frame->can_dlc==4)
  {
    int16_t*   data = (int16_t*)frame->data;
    _temperature    = ((float)data[0]) / 100.f;
    uint16_t* udata = (uint16_t*)(&(frame->data[2]));
    _voltageSys     = ((float)udata[0]) / 100.f;
    
    if(_verbosity)
      std::cout << "T=" << _temperature << "°C Vsys=" << _voltageSys << std::endl;
      
    _init = true;
  }
}

} // namespace