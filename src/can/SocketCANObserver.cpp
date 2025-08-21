#include "SocketCANObserver.h"

namespace edu
{

SocketCANObserver::SocketCANObserver() :
  _canid(0x0),
  _last_msg_stamp(std::chrono::steady_clock::now())
{
  
}

SocketCANObserver::~SocketCANObserver()
{

}

void SocketCANObserver::setCANId(canid_t canid)
{
  _canid = canid;
}

canid_t SocketCANObserver::getCANId()
{
  return _canid;
}

void SocketCANObserver::forwardNotification(struct can_frame* frame)
{
  if(frame->can_id == _canid)
  {
    _last_msg_stamp = std::chrono::steady_clock::now();
	  notify(frame);
  }
}

bool SocketCANObserver::checkConnectionStatus(unsigned int timeout_ms)
{
    auto now = std::chrono::steady_clock::now();
    return (now - _last_msg_stamp) < std::chrono::milliseconds(timeout_ms);
}

}
