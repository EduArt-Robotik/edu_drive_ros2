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
   * @brief Set angle of servo motor
   * @param[in] channel Channel of servo motor connector
   * @param[in] angle Desired angle
   * @return success state
   */
  bool setServo(int channel, double angle);
  

private:

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;     // Input address (CAN ID) of carrier board

    int32_t          _outputAddress;    // Output address (CAN ID) of carrier board

    int32_t          _broadcastAddress; // Broadcast address for the distribution of CAN data to multiple nodes
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
};

} // namespace

#endif // _RPIEXTENSIONBOARD_H_
