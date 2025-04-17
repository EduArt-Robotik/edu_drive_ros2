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

bool RPiExtensionBoard::setServos(int bank, int channels[4], double angles[4])
{
  if(bank==0 || bank==1)
  {
    _cf.can_dlc = 5;
    _cf.data[0] = bank; // bank 0 relates to servos 1 to 4, bank 1 to servos 5 to 8
    for(int ch=0; ch<4; ch++)
    {
      _cf.data[ch+1] = 255; // this values means, to not change servo position (values > 250 are ignored)
      if(channels[ch]>0 && channels[ch]<=4)
      {
        uint8_t uAngle = uint8_t((angles[ch] / 270.f) * 250.f);
        _cf.data[channels[ch]] = uAngle;
      }
    }
    return _can->send(&_cf);
  }
  else
  {
    return false;
  }
}

} // namespace
