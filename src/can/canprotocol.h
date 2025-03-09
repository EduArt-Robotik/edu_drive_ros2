/*
 * canprotocol.h
 *
 *  Created on: Mar 20, 2021
 *      Author: Stefan May
 */

#ifndef CANIDS_H_
#define CANIDS_H_

#include <stdint.h>

namespace edu
{

#define INPUTBIT          0b0
#define OUTPUTBIT         0b1
#define SYSID_MC          0b001 // Motorcontroller
#define SYSID_LIGHT       0b010 // Lighting shields with ToF sensors
#define SYSID_RPI_ADAPTER 0b011
#define SYSID_MC2         0b100 // Motorcontroller (Version for RPi4 and IOT)
#define SYSID_PWRMGMT     0b101

#define RPI_ADAPTER      0b0000001
#define RPI_EXTENSION    0b0000010
#define IOT_SHIELD       0b0000001
#define NODEID_HEADLEFT  0b0000001
#define NODEID_HEADRIGHT 0b0000010
#define NODEID_TAILLEFT  0b0000011
#define NODEID_TAILRIGHT 0b0000100
#define NODEID_PWRMGMT   0b0000000

/**
 * Power management commands
 */
// Common parameters
#define CMD_PWR_MGMT_ENABLE       0x01
#define CMD_PWR_MGMT_DISABLE      0x02

/**
 * Motor commands
 */
// Common parameters
#define CMD_MOTOR_ENABLE          0x01
#define CMD_MOTOR_DISABLE         0x02
#define CMD_MOTOR_SETTIMEOUT      0x03
#define CMD_MOTOR_SETPWMMAX       0x04
#define CMD_MOTOR_SENDRPM         0x05
#define CMD_MOTOR_SENDPOS         0x06
#define CMD_MOTOR_INVERTENC       0x07

// Operating commands
#define CMD_MOTOR_SETPWM          0x10
#define CMD_MOTOR_SETRPM          0x11
#define CMD_MOTOR_FREQ_SCALE      0x12
#define CMD_MOTOR_SYNC            0x13

// Closed/Open loop controller parameters
#define CMD_MOTOR_CTL_KP          0x20
#define CMD_MOTOR_CTL_KI          0x21
#define CMD_MOTOR_CTL_KD          0x22
#define CMD_MOTOR_CTL_ANTIWINDUP  0x23
#define CMD_MOTOR_CTL_INPUTFILTER 0x24

// Platform parameters
#define CMD_MOTOR_GEARRATIO       0x30
#define CMD_MOTOR_GEARRATIO2      0x31
#define CMD_MOTOR_TICKSPERREV     0x32
#define CMD_MOTOR_TICKSPERREV2    0x33

// Standard responses
#define RESPONSE_MOTOR_RPM        0xA0
#define RESPONSE_MOTOR_POS        0xA1

// Error responses
#define ERR_ENCA_NOSIGNAL   0xE0
#define ERR_ENCB_NOSIGNAL   0xE1


/**
 * Lighting commands
 */
#define CAN_LIGHT_BEAT        1   // Heartbeat of IOTShield for synchronizing attached devices
#define CAN_LIGHT_LIGHTS_OFF  2   // Lights off
#define CAN_LIGHT_DIM_LIGHT   3   // Dimmed headlight
#define CAN_LIGHT_HIGH_BEAM   4   // high beam headlight
#define CAN_LIGHT_FLASH_ALL   5   // Flash lights
#define CAN_LIGHT_FLASH_LEFT  6   // Flash lights to the left
#define CAN_LIGHT_FLASH_RIGHT 7   // Flash lights to the right
#define CAN_LIGHT_PULSATION   8   // Pulsation
#define CAN_LIGHT_ROTATION    9   // Rotating light
#define CAN_LIGHT_RUNNING     10  // Running light


/**
 * IMU commands
 */
#define CAN_QUATERNION  100 // Quaternion representing orientation

void makeCanStdID(int32_t sysID, int32_t nodeID, int32_t* inputAddress, int32_t* outputAddress, int32_t* broadcastAddress);

} // namespace

#endif /* CANIDS_H_ */

