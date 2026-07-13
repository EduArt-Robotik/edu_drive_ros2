#include <iostream>

#define VOLTAGE_BAT_INIT        19.5
#define VOLTAGE_SUPPLY          29.5
#define VOLTAGE_LOAD_DECREASE   -0.02
#define VOLTAGE_SUPPLY_INCREASE 0.02

/**
 * This programm simulates the enable logic for the EduArt power management module
 * @author Stefan May
 * @date 07.02.2024
 */
int main(int argc, char* argv[])
{
  // timestep in seconds
  double timestep = 1.0;

  // global time
  double time = 0.0;

  // run simulation as long as this variable is true
  bool simulationOn = true;

  // battery voltage
  double voltage = VOLTAGE_BAT_INIT;

  // system voltage
  double voltageSystem = VOLTAGE_BAT_INIT;

  // flag indicating that battery is loaded
  bool loading = false;

  // a load reduces the battery voltage per timestep as long as the power supply is not attached
  // attaching the power supply increases the voltage per timestep by this factor
  double voltageIncrease = VOLTAGE_LOAD_DECREASE;

  // Enable state
  bool enabled = false;

  // time for measuring first the low voltage event
  double timeLowVoltage = 0.0;

  int state = 0;

  while (simulationOn)
  {

    switch (state)
    {
    case 0:
      if (voltage < 17.8)
        state = 1;
      break;
    case 1:
      loading = true;
      if (voltage > 19.0)
        state = 2;
      break;
    case 2:
      loading = false;
      if (voltage < 17.0)
        state = 3;
      break;
    case 3:
      loading = true;
      if (voltage > 18.6)
        state = 4;
      break;
    case 4:
      loading = false;
      break;
    }

    std::cout << time << " " << timeLowVoltage << " ";

    if (loading)
    {
      voltageIncrease = VOLTAGE_SUPPLY_INCREASE;
      voltageSystem   = VOLTAGE_SUPPLY;
    }
    else
    {
      voltageIncrease = VOLTAGE_LOAD_DECREASE;
      voltageSystem   = voltage;
    }
    voltage += voltageIncrease * timestep;

    // print enable state

    if (voltageSystem < 24.0 && voltageSystem > 17.5)
    {
      enabled        = true;
      timeLowVoltage = 0.0;
    }
    else if (voltageSystem > 24.0)
    {
      enabled        = false;
      timeLowVoltage = 0.0;
    }
    else
    {
      if (timeLowVoltage > 10.0)
        enabled = false;
      if (timeLowVoltage > 120.0)
        voltageSystem = 0.0;
      if (timeLowVoltage > 130.0)
        simulationOn = false;
      timeLowVoltage += timestep;
    }

    std::cout << voltageSystem << " " << enabled << " 17.5 24.0" << std::endl;

    time += timestep;
  }
}
