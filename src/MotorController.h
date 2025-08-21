#pragma once

#include "can/SocketCAN.h"
#include <vector>

namespace edu
{

  enum CanResponse
  {
    CAN_RESPONSE_RPM = 0,
    CAN_RESPONSE_POS = 1
  };

  struct MotorParams
  {
    // Channel of motor controller interface
    int channel;

    // Kinematic vector
    std::vector<double> kinematics;

    enum CanResponse responseMode;
    
    int invertEnc;
    float gearRatio;
    float encoderRatio;
    float rpmMax;
    
    MotorParams()
    {
      channel       = 0;
      invertEnc     = 0;
      gearRatio     = 0.0f;
      encoderRatio  = 0.0f;
      rpmMax        = 0.0f;
      kinematics.clear();
      responseMode = CAN_RESPONSE_RPM;
    }

    /**
     * Copy constructor
     * @param[in] p parameter instance to be copied
     */
    MotorParams(const MotorParams &p)
    {
      channel       = p.channel;
      kinematics    = p.kinematics;
      responseMode  = p.responseMode;
      gearRatio     = p.gearRatio;
      encoderRatio  = p.encoderRatio;
      rpmMax        = p.rpmMax;
      invertEnc     = p.invertEnc;
    }
  };

  struct ControllerParams
  {
    // CAN interface ID
    int canID;

    // Control parameters
    unsigned short frequencyScale;
    float inputWeight;
    unsigned char maxPulseWidth;
    unsigned short timeout;
    float kp;
    float ki;
    float kd;
    int antiWindup;

    CanResponse responseMode;
    std::vector<MotorParams> motorParams;

    /**
     * Standard constructor assigns default parameters
     */
    ControllerParams()
    {
      canID = 0;
      frequencyScale = 32;
      inputWeight = 0.8f;
      maxPulseWidth = 50;
      timeout = 300;
      kp = 0.f;
      ki = 0.f;
      kd = 0.f;
      antiWindup = 1;
      responseMode = CAN_RESPONSE_RPM;

      motorParams.resize(2);
      std::vector<double> zeroKinematic{0.f, 0.f, 0.f};
      motorParams[0].channel = 0;
      motorParams[0].kinematics = zeroKinematic;
      motorParams[1].channel = 1;
      motorParams[1].kinematics = zeroKinematic;
    }

    /**
     * Copy constructor
     * @param[in] p parameter instance to be copied
     */
    ControllerParams(const ControllerParams &p)
    {
      canID = p.canID;
      frequencyScale = p.frequencyScale;
      inputWeight = p.inputWeight;
      maxPulseWidth = p.maxPulseWidth;
      timeout = p.timeout;
      kp = p.kp;
      ki = p.ki;
      kd = p.kd;
      antiWindup = p.antiWindup;
      responseMode = p.responseMode;
      motorParams = p.motorParams;
    }
  };

  /**
   * @class MotorControllerCAN
   * @brief CAN interface for EduArt's robot motor controller.
   * @author Stefan May
   * @date 27.04.2022
   */
  class MotorController : public SocketCANObserver
  {
  public:
    /**
     * Constructor
     * @param[in] can SocketCAN instance
     * @param[in] params controller parameters
     * @param[in] verbosity verbosity output flag
     */
    MotorController(SocketCAN *can, ControllerParams params, bool verbosity = 0);

    /**
     * Destructor
     */
    ~MotorController();

    /**
     * Check whether device is initialized after powering on
     */
    bool isInitialized();

    /**
     * Revert initialization state
     */
    void deinit();

    /**
     * Reinitialize device
     */
    void reinit();

    /**
     * Enable device
     * @return successful transmission of enable command
     */
    bool enable();

    /**
     * Disable device
     * @return successful transmission of disabling command
     */
    bool disable();

    /**
     * Get state of motor controller (enabled / disabled)
     * @return enable state
     */
    bool getEnableState();

    /**
     * Send synchronization signal, resetting a counter in order to have a common time basis on all controllers
     * @return successful transmission of synchronization signal
     */
    bool broadcastExternalSync();

    /**
     * @brief Get the parameters of connected motors
     *
     * @return MotorParams vector
     */
    std::vector<MotorParams> getMotorParams();

    /**
     * Configure response of motor controller (revolutions per minute or position)
     * @param[in] mode response mode
     * @return successful transmission of configure command
     */
    bool configureResponse(enum CanResponse mode);

    /**
     * Invert encoder polarity
     * @param[in] invert set to true, if channels need to be inverted
     * @return successful transmission of configure command
     */
    bool invertEncoderPolarity(bool invert[2]);

    /**
     * Get assigned canID via constructor
     * @return ID
     */
    unsigned short getCanId();

    /**
     * Set timeout interval. The motor controller needs frequently transmitted commands.
     * If the time span between two commands is longer than this timeout interval, the device is disabled.
     * The user needs to send an enabling command again.
     * @param[in] timeoutInMillis timeout interval in milliseconds
     * @return true==successful CAN transmission
     */
    bool setTimeout(unsigned short timeoutInMillis);

    /**
     * Accessor to timeout parameter. See commets of mutator for more information.
     * @return timeout in milliseconds
     */
    unsigned short getTimeout();

    /**
     * Set gear ratio (factor between motor and wheel revolutions)
     * @param[in] gearRatio (motor rev) / (wheel rev) for motor 1 and 2 separately
     * @return true==successful CAN transmission
     */
    bool setGearRatio(float gearRatio[2]);

    /**
     * Accessor to gear ratio parameter
     * @return gearRatio (motor rev) / (wheel rev) for motor 1 and 2
     */
    float getGearRatio(size_t motor_num);

    /**
     * Set number of encoder ticks per motor revolution
     * @param[in] encoderTicksPerRev encoder ticks per motor revolution for motor 1 and 2
     * @return true==successful CAN transmission
     */
    bool setEncoderTicksPerRev(float encoderTicksPerRev[2]);

    /**
     * Accessor to parameter representing encoder ticks per motor revolution
     * @return encoder ticks per motor revolution for motor 1 and 2
     */
    float getEncoderTicksPerRev(size_t motor_num);

    /**
     * Set scaling parameter for PWM frequency. The base frequency is 500kHz, of which one can apply a fractional amount, e.g.
     * 10 => 50kHz
     * 20 => 25kHz
     * Default is 32 => 15,625kHz
     * Important: This value can only be changed before the motor controllers gets enabled.
     * @param[in] scale denominator d of term 1/d x 500kHz
     */
    bool setFrequencyScale(unsigned short scale);

    /**
     * Accessor to frequency scaling parameter, see mutator for details.
     * @return scale denominator d of term 1/d x 500kHz
     */
    unsigned short getFrequencyScale();

    /**
     * The PWM signal can be adjusted in the range from [-127;127] which is equal to [-100%;100%].
     * To limit the possible output, one can set a different value between [0;127],
     * which is symmetrically applied to the positive and negative area, e.g. 32 => [-25%;25%]
     * The default value is: 63 => [-50%;50%]
     * @param[in] pulse pulse width limit in range of [-127;127]
     */
    bool setMaxPulseWidth(unsigned char pulse);

    /**
     * Accessor to pulse width limit, see mutator for details.
     * @return pulse width limit
     */
    unsigned char getMaxPulseWidth();

    /**
     * Set pulse width modulated signal
     * @param[in] rpm pulse width in range [-100;100], this device supports 2 channels
     * @return success
     */
    bool setPWM(int pwm[2]);

    /**
     * Set motor revolutions per minute
     * @param[in] rpm set point value, this device supports 2 channels
     * @return success
     */
    bool setRPM(float rpm[2]);

    /**
     * Get either motor revolutions per minute or motor position (encoder ticks). This depends on the configuration canResponseMode.
     * @param[out] response revolutions per minute for motor 1 and 2 / position of motor 1 and 2. This is a modulo 2^15 value.
     */
    void getWheelResponse(float response[2]);

    /**
     * Set proportional factor of PID controller
     * @param[in] kp proportional factor
     * @return success
     */
    bool setKp(float kp);

    /**
     * Accessor to proportional factor of PID controller
     * @return proportional factor
     */
    float getKp();

    /**
     * Set integration factor of PID controller
     * @param[in] ki integration factor
     * @return success
     */
    bool setKi(float ki);

    /**
     * Accessor to integration factor of PID controller
     * @return integration factor
     */
    float getKi();

    /**
     * Set differential factor of PID controller
     * @param[in] kd differential factor
     * @return success
     */
    bool setKd(float kd);

    /**
     * Accessor to differential factor of PID controller
     * @return differential factor
     */
    float getKd();

    /**
     * Activate/Deactivate anti windup functionality of PID controller
     * @param[in] activate activation==true, deactivation==false
     * @return success
     */
    bool setAntiWindup(bool activate);

    /**
     * Accessor to anti windup parameter of PID controller
     * @return anti windup activation
     */
    bool getAntiWindup();

    /**
     * Set weight of input filter. Input values f are filtered with f'=weight*f'+(1-weight)*f.
     * @param[in] filtering weight. A value of 0 disables the filter. The value must be in the range of [0;1[
     */
    bool setInputWeight(float weight);

    /**
     * Accessor to weight of input filter. See comments of mutator for more information.
     * @return weight of input filter
     */
    float getInputWeight();

    /**
     * Stop motors
     */
    void stop();

  protected:
  private:
  
    /**
     * Initialize motor controllers (adjust parameters)
     **/
    void init();
    
    bool sendFloat(int cmd, float f);

    /**
     * Implementation of inherited method from SocketCANObserver. This class is getting notified by the SocketCAN,
     * as soon as messages of interest arrive (having the desired CAN ID).
     * @param[in] frame CAN frame
     */
    void notify(struct can_frame *frame);

    SocketCAN *_can;

    int32_t _inputAddress;

    int32_t _outputAddress;

    int32_t _broadcastAddress;

    can_frame _cf;

    ControllerParams _params;

    bool _isInit;

    float _rpm[2];

    short _pos[2];

    bool _enabled;

    bool _verbosity;

    long _seconds;

    long _usec;
  };

}
