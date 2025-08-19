#pragma once

#include "can/SocketCAN.h"

namespace edu
{

/**
 * @class CarrierBoard
 * @brief Interface to EduArt's robot power management board.
 * @author Hannes Duske
 * @date 01.02.2024
 */
class PowerManagementBoard : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] params motor parameters
   * @param[in] verbosity verbosity output flag
   */
  PowerManagementBoard(SocketCAN* can, bool verbosity=false);

  /**
   * Destructor
   */
  ~PowerManagementBoard();

    /**
   * Set hardware enable signal
   * @return successful transmission of enable command
   */
  bool enable();

  /**
   * Reset hardware enable signal
   * @return successful transmission of disabling command
   */
  bool disable();
  
  /**
   * @brief Get voltage of main power rail
   * @return voltage [V]
   */
  float getVoltage();
  
  /**
   * @brief Get current consumed by robot
   * @return current [A]
   */
  float getCurrent();

private:

    void notify(struct can_frame* frame);

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;     // Input address (CAN ID) of carrier board

    int32_t          _outputAddress;    // Output address (CAN ID) of carrier board

    int32_t          _broadcastAddress; // Broadcast address for the distribution of CAN data to multiple nodes

    float            _voltage;          // Voltage of main power rail

    float            _current;          // Current consumed by Robot
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
    bool             _init;
};

}
