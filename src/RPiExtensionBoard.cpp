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

  //canid_t canidOutput = _outputAddress;

  //setCANId(canidOutput);
 
}

RPiExtensionBoard::~RPiExtensionBoard()
{

}

bool RPiExtensionBoard::setServo(int channel, double angle)
{
  _cf.can_dlc = 1;
  uint8_t uAngle = uint8_t((angle / 270.f) * 255.f);
  _cf.data[0] = uAngle;
  return _can->send(&_cf);
}

} // namespace
