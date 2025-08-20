#include "SocketCAN.h"

#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <cstring>

using namespace std::chrono_literals;
using LockGuard = std::lock_guard<std::mutex>;

namespace edu
{

SocketCAN::SocketCAN(std::string devFile) :
  _soc(0),
  _listenerIsRunning(false),
  _shutDownListener(false),
  _portOpen(false),
  _next_time(std::chrono::steady_clock::now())
{
  _portOpen = openPort(devFile.c_str());
  if(!_portOpen)
    std::cout << "ERROR: Cannot open CAN device interface: " << devFile << std::endl;
}

SocketCAN::~SocketCAN()
{
  stopListener();
  closePort();
}

bool SocketCAN::registerObserver(SocketCANObserver* observer)
{
  LockGuard guard(_mutex);
  _observers.push_back(observer);
  return true;
}

void SocketCAN::clearObservers()
{
  LockGuard guard(_mutex);
  _observers.clear();
}

bool SocketCAN::openPort(const char *port)
{
  struct ifreq ifr;
  struct sockaddr_can addr;

  _soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(_soc < 0) return false;

  addr.can_family = AF_CAN;
  std::strcpy(ifr.ifr_name, port);

  if (ioctl(_soc, SIOCGIFINDEX, &ifr) < 0) return false;

  addr.can_ifindex = ifr.ifr_ifindex;

  fcntl(_soc, F_SETFL, O_NONBLOCK);

  if (bind(_soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;

  return true;
}

bool SocketCAN::send(struct can_frame* frame)
{
  // wait until the next slot
  std::this_thread::sleep_until(_next_time);
  _next_time = std::chrono::steady_clock::now() + 2ms;

  int retval;
  {
    LockGuard guard(_mutex);
    retval = write(_soc, frame, sizeof(struct can_frame));
  }
  if (retval != sizeof(struct can_frame))
  {
    std::cout << "Can transmission error for command " << (int)(frame->data[0]) << ", returned " << retval << " submitted bytes instead of " << sizeof(struct can_frame) << std::endl;
    return false;
  }

  return true;
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

  std::cout << "CAN listener start" << std::endl;

  _listenerIsRunning = true;
  while(!_shutDownListener)
  {
    FD_ZERO(&readSet);

    {
      LockGuard guard(_mutex);
      FD_SET(_soc, &readSet);
      if (select((_soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
      {
        if (FD_ISSET(_soc, &readSet))
        {
          recvbytes = read(_soc, &frame_rd, sizeof(struct can_frame));
          if(recvbytes)
          {
            for(auto observer : _observers)
            {
              observer->forwardNotification(&frame_rd);
            }
          }
        }
      }
    }

    std::this_thread::sleep_for(100us);
  }

  std::cout << "CAN listener stop" << std::endl;

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
