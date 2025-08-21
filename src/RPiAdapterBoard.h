#pragma once

#include "can/SocketCAN.h"

namespace edu
{

/**
 * @class RPiAdapterBoard
 * @brief Interface to EduArt's robot RPi adapter board.
 * @author Stefan May
 * @date 27.04.2022
 */
class RPiAdapterBoard : public SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] can SocketCAN instance
   * @param[in] params motor parameters
   * @param[in] verbosity verbosity output flag
   */
  RPiAdapterBoard(SocketCAN* can, bool verbosity=false);

  /**
   * Destructor
   */
  ~RPiAdapterBoard();
  
  /**
   * @brief Get orientation as quaternion
   * @param[out] q Orientation quaternion (layout: [w x y z])
   */
  void getOrientation(double q[4]);
  
  /**
   * @brief Get temperature of carrier board
   * @return temperature in degree Celsius
   */
  double getTemperature();

  /**
   * @brief Get system voltage (provided by supply pins)
   * @return voltage [V]
   */
  double getVoltageSys();

  /**
   * @brief Get raw acceleration values of IMU
   * @param[out] acc Acceleration in x-, y- and z-dimension
   */
  void getAcceleration(double acc[3]);

    /**
   * @brief Get raw angular velocity values of IMU
   * @param[out] vel Angular velocity in x-, y- and z-dimension
   */
  void getAngularVelocity(double vel[3]);

private:

    void notify(struct can_frame* frame);

    SocketCAN*       _can;

    can_frame        _cf;

    int32_t          _inputAddress;     // Input address (CAN ID) of carrier board

    int32_t          _outputAddress;    // Output address (CAN ID) of carrier board

    int32_t          _broadcastAddress; // Broadcast address for the distribution of CAN data to multiple nodes

    double           _q[4];             // Orientation data as quaternion (layout [w x y z])

    double           _temperature;      // Temperature of surface of carrier board

    double           _voltageSys;       // Voltage supply of adapter board

    double           _acceleration[3];  // Acceleration in x-, y-, z-dimension.

    double           _angular_velocity[3];  // Acceleration in x-, y-, z-dimension.
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
    bool             _init;
};

}
