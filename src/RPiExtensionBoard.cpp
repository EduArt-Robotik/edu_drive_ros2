#include "RPiExtensionBoard.h"
#include <iostream>
#include "can/canprotocol.h"
#include <unistd.h>

namespace edu
{

RPiExtensionBoard::RPiExtensionBoard(SocketCAN* can, bool verbosity)
{
  _verbosity = verbosity;
  _can = can;
  
  makeCanStdID(SYSID_RPI_ADAPTER, RPI_EXTENSION, &_inputAddress, &_outputAddress, &_broadcastAddress);
  _cf.can_id = _inputAddress;
  
  if(verbosity)
    std::cout << "#RPiExtensionBoard CAN Input ID: " << std::hex << _inputAddress << " CAN Output ID: " << _outputAddress << std::endl;
 
}

RPiExtensionBoard::~RPiExtensionBoard()
{

}

bool RPiExtensionBoard::setServos(double angles[8])
{
  _cf.can_dlc = 8;
  for(int ch=0; ch<8; ch++)
  {
    uint8_t uAngle = uint8_t((angles[ch] / 270.f) * 250.f);
    _cf.data[ch] = uAngle;
  }
  return _can->send(&_cf);
}

} // namespace
