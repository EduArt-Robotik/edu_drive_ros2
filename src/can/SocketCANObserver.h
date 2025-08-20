#pragma once

#include <linux/can.h>
#include <chrono>

namespace edu
{

/**
 * @class SocketCANObserver
 * @brief Abstract observer class. Derived classes get notified according their CAN bus identifiers.
 * @author Stefan May
 * @date 13.05.2018
 */
class SocketCANObserver
{
public:
  /**
   * Constructor
   */
  SocketCANObserver();

  /**
   * Destructor
   */
  virtual ~SocketCANObserver();

  /**
   * Set CAN bus identifier
   * @param[in] id CAN ID
   */
  void setCANId(canid_t id);

  /**
   * Get CAN bus identifier
   * @return CAN ID
   */
  canid_t getCANId();

  /**
   * Check connection status, i.e., whether the elapsed time since the last message arrival is smaler than a specific timeout.
   * @param[in] timeout_ms timeout in milliseconds
   * @return connection status
   */
  bool checkConnectionStatus(unsigned int timeout_ms=100);
  
  /**
   * Notify the observer with new message
   * @param[in] frame CAN Frame
   */
  void forwardNotification(struct can_frame* frame);

protected:
  /**
   * Interface declaration for implementation through inherited classes.
   * @param[in] frame CAN frame
   */
  virtual void notify(struct can_frame* frame) = 0;

private:

  canid_t _canid;

  std::chrono::time_point<std::chrono::steady_clock> _last_msg_stamp;

};

}
