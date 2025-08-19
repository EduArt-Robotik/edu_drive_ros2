#include "SocketCANObserver.h"
#include <sys/time.h>

namespace edu
{

SocketCANObserver::SocketCANObserver()
{
  _canid = 0x0;
  _seconds = 0;
  _usec = 0;
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
    // Take time stamp
    timeval clock;
    ::gettimeofday(&clock, 0);
    _seconds = clock.tv_sec;
    _usec = clock.tv_usec;
    
	notify(frame);
}

bool SocketCANObserver::checkConnectionStatus(unsigned int timeoutInMillis)
{
      // Take time stamp
    timeval clock;
    ::gettimeofday(&clock, 0);
    long seconds = clock.tv_sec;
    long usec = clock.tv_usec;

    long deltaSec  = seconds - _seconds;
    long deltaUSec = usec - _usec;

    long elapsed = deltaSec * 1000 + deltaUSec / 1000;
    return elapsed < timeoutInMillis;
}

}
