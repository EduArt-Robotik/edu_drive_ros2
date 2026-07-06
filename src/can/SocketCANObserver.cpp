#include "SocketCANObserver.h"

namespace edu
{

namespace
{
using SteadyClock = std::chrono::steady_clock;
using Nanoseconds = std::chrono::nanoseconds;
static constexpr std::int64_t MS2NS = 1000000;

std::int64_t nowNs()
{
  return std::chrono::duration_cast<Nanoseconds>(SteadyClock::now().time_since_epoch()).count();
}
} // namespace

SocketCANObserver::SocketCANObserver() :
  _canid(0x0),
  _last_msg_stamp_ns(nowNs())
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
    _last_msg_stamp_ns.store(nowNs(), std::memory_order_relaxed);
    notify(frame);
  }
}

bool SocketCANObserver::checkConnectionStatus(unsigned int timeout_ms)
{
  auto now_ns = nowNs();
  auto last_ns = _last_msg_stamp_ns.load(std::memory_order_relaxed);
  auto timeout_ns = static_cast<std::int64_t>(timeout_ms) * MS2NS;
  return (now_ns - last_ns) < timeout_ns;
}

}
