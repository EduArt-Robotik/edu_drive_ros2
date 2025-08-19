#include "SocketCAN.h"

#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace edu
{

SocketCAN::SocketCAN(std::string devFile)
{
  _soc = 0;
  _listenerIsRunning = false;
  _shutDownListener  = false;
  _portOpen = openPort(devFile.c_str());
  if(!_portOpen)
    std::cout << "WARNING: Cannot open CAN device interface: " << devFile << std::endl;
}

SocketCAN::~SocketCAN()
{
  stopListener();
  closePort();
}

bool SocketCAN::registerObserver(SocketCANObserver* observer)
{
  _mutex.lock();
  _observers.push_back(observer);
  _mutex.unlock();
  return true;
}

void SocketCAN::clearObservers()
{
  _mutex.lock();
  _observers.clear();
  _mutex.unlock();
}

bool SocketCAN::openPort(const char *port)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  _soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(_soc < 0)
  {
    return false;
  }

  addr.can_family = AF_CAN;
  std::strcpy(ifr.ifr_name, port);

  if (ioctl(_soc, SIOCGIFINDEX, &ifr) < 0)
  {
    return false;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  fcntl(_soc, F_SETFL, O_NONBLOCK);

  if (bind(_soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    return false;
  }

  return true;
}

bool SocketCAN::send(struct can_frame* frame)
{
  static double _timeCom = 0.0;
  timeval clock;
  double now = 0.0;
  do
  {
     ::gettimeofday(&clock, 0);
     now = static_cast<double>(clock.tv_sec) + static_cast<double>(clock.tv_usec) * 1.0e-6;
  }while((now - _timeCom) < 0.002);
  _timeCom = now;
    
  int retval;
  _mutex.lock();
  retval = write(_soc, frame, sizeof(struct can_frame));
  _mutex.unlock();
  if (retval != sizeof(struct can_frame))
  {
    std::cout << "Can transmission error for command " << (int)(frame->data[0]) << ", returned " << retval << " submitted bytes instead of " << sizeof(struct can_frame) << std::endl;
    return false;
  }
  else
  {
    return true;
  }
}

bool SocketCAN::startListener(int timeout_ms)
{
  if(!_portOpen || _listenerIsRunning) return false;

  _thread = std::make_unique<std::thread>(&SocketCAN::listener, this);

  int watchdog = 0;
  while(!_listenerIsRunning && (watchdog < timeout_ms))
  {
    std::this_thread::sleep_for(1ms);
    watchdog += 1;
  }

  return watchdog < timeout_ms;
}

bool SocketCAN::listener()
{
  _shutDownListener  = false;

  struct can_frame frame_rd;
  int recvbytes = 0;

  struct timeval timeout = {0, 100};
  fd_set readSet;

  std::cout << "# Listener start" << std::endl;

  _listenerIsRunning = true;
  while(!_shutDownListener)
  {
    FD_ZERO(&readSet);

    _mutex.lock();
    FD_SET(_soc, &readSet);
    if (select((_soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
    {
      if (FD_ISSET(_soc, &readSet))
      {
        recvbytes = read(_soc, &frame_rd, sizeof(struct can_frame));
        if(recvbytes)
        {
          for(std::vector<SocketCANObserver*>::iterator it=_observers.begin(); it!=_observers.end(); ++it)
          {
            if((*it)->getCANId()==frame_rd.can_id)
              (*it)->forwardNotification(&frame_rd);
          }
        }
      }
    }
    _mutex.unlock();

    std::this_thread::sleep_for(100us);
  }
  std::cout << "# Listener stop" << std::endl;

  _listenerIsRunning = false;
  return true;
}

void SocketCAN::stopListener()
{
  _shutDownListener = true;
  _thread->join();
}

bool SocketCAN::closePort()
{
  bool retval = false;
  if(_soc)
  {
    retval = (close(_soc)==0);
  }
  return retval;
}

}
