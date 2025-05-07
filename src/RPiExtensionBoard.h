#ifndef _RPIEXTENSIONBOARD_H_
#define _RPIEXTENSIONBOARD_H_

#include "can/SocketCAN.h"

namespace edu
{

/**
 * @class RPiExtensionBoard
 * @brief Interface to EduArt's robot RPi extension board.
 * @author Stefan May
 * @date 09.03.2025
 */
class RPiExtensionBoard
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] params motor parameters
   * @param[in] verbosity verbosity output flag
   */
  RPiExtensionBoard(SocketCAN* can, bool verbosity=false);

  /**
   * Destructor
   */
  ~RPiExtensionBoard();
  
  /**
   * @brief Set angle of servo motors
   * @param[in] angle Desired angle, valid values: [0, 270]
   * @return success state of CAN transmission
   */
  bool setServos(double angles[8]);
  
  /**
   * @brief Send enable state for visualization (LEDs)
   * @param[in] enabled joint enabled state of all motors
   * @return success state of CAN transmission
   */
  bool sendEnabledState(bool enabled);
  

private:

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;     // Input address (CAN ID) of extension board

    int32_t          _outputAddress;    // Output address (CAN ID) of extension board

    int32_t          _broadcastAddress; // Broadcast address for the distribution of CAN data to multiple nodes
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
};

} // namespace

#endif // _RPIEXTENSIONBOARD_H_
