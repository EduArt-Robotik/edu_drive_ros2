#include <iostream>

#include "EduDrive.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>

int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);
   auto edu_drive = std::make_shared<edu::EduDrive>();

   std::vector<edu::ControllerParams> controllerParams;

   // --- System parameters --------
   bool usingPwrMgmt;
   std::string canInterface;
   int frequencyScale;
   float inputWeight;
   int maxPulseWidth;
   int timeout;
   
   edu_drive->declare_parameter("usingPowerManagementBoard", true);
   edu_drive->declare_parameter("canInterface",              std::string("can0"));
   edu_drive->declare_parameter("frequencyScale",            32);
   edu_drive->declare_parameter("inputWeight",               0.8f);
   edu_drive->declare_parameter("maxPulseWidth",             50);
   edu_drive->declare_parameter("timeout",                   300);

   usingPwrMgmt   = edu_drive->get_parameter("usingPowerManagementBoard").as_bool();
   canInterface   = edu_drive->get_parameter("canInterface").as_string();
   frequencyScale = edu_drive->get_parameter("frequencyScale").as_int();
   inputWeight    = edu_drive->get_parameter("inputWeight").as_double();
   maxPulseWidth  = edu_drive->get_parameter("maxPulseWidth").as_int();
   timeout        = edu_drive->get_parameter("timeout").as_int();

   // Ensure a proper range for the timeout value
   // A lag more than a second should not be tolerated
   if (timeout < 0 && timeout > 1000)
      timeout = 300;

   float kp;
   float ki;
   float kd;
   int antiWindup;
   int responseMode;

   edu_drive->declare_parameter("kp", 0.0f);
   edu_drive->declare_parameter("ki", 0.f);
   edu_drive->declare_parameter("kd", 0.f);
   edu_drive->declare_parameter("antiWindup", 1);
   edu_drive->declare_parameter("responseMode", 0);

   kp = edu_drive->get_parameter("kp").as_double();
   ki = edu_drive->get_parameter("ki").as_double();
   kd = edu_drive->get_parameter("kd").as_double();
   antiWindup   = edu_drive->get_parameter("antiWindup").as_int();
   responseMode = edu_drive->get_parameter("responseMode").as_int();

   // -----------------------------

   // --- Controller parameters ---
   int controllers = 0;
   edu_drive->declare_parameter("controllers", 0);
   controllers = edu_drive->get_parameter("controllers").as_int();

   for(int c=0; c<controllers; c++)
   {
      edu::ControllerParams cp;

      std::string controllerID = std::string("controller") + std::to_string(c);
      edu_drive->declare_parameter(controllerID + std::string(".canID"), 0);
      edu_drive->declare_parameter(controllerID + std::string(".gearRatio"), 0.f);
      edu_drive->declare_parameter(controllerID + std::string(".encoderRatio"), 0.f);
      edu_drive->declare_parameter(controllerID + std::string(".rpmMax"), 0.f);
      edu_drive->declare_parameter(controllerID + std::string(".invertEnc"), 0);
      edu_drive->declare_parameter(controllerID + std::string(".frequencyScale"), frequencyScale);
      edu_drive->declare_parameter(controllerID + std::string(".inputWeight"), inputWeight);
      edu_drive->declare_parameter(controllerID + std::string(".maxPulseWidth"), maxPulseWidth);
      edu_drive->declare_parameter(controllerID + std::string(".timeout"), timeout);
      edu_drive->declare_parameter(controllerID + std::string(".kp"), kp);
      edu_drive->declare_parameter(controllerID + std::string(".ki"), ki);
      edu_drive->declare_parameter(controllerID + std::string(".kd"), kd);
      edu_drive->declare_parameter(controllerID + std::string(".antiWindup"), antiWindup);

      cp.canID        = edu_drive->get_parameter(controllerID + std::string(".canID")).as_int();
      cp.gearRatio    = edu_drive->get_parameter(controllerID + std::string(".gearRatio")).as_double();
      cp.encoderRatio = edu_drive->get_parameter(controllerID + std::string(".encoderRatio")).as_double();
      cp.rpmMax       = edu_drive->get_parameter(controllerID + std::string(".rpmMax")).as_double();
      cp.invertEnc    = edu_drive->get_parameter(controllerID + std::string(".invertEnc")).as_int();
      cp.frequencyScale=edu_drive->get_parameter(controllerID + std::string(".frequencyScale")).as_int();
      cp.inputWeight  = edu_drive->get_parameter(controllerID + std::string(".inputWeight")).as_double();
      cp.maxPulseWidth= edu_drive->get_parameter(controllerID + std::string(".maxPulseWidth")).as_int();
      cp.timeout      = edu_drive->get_parameter(controllerID + std::string(".timeout")).as_int();
      cp.kp           = edu_drive->get_parameter(controllerID + std::string(".kp")).as_double();
      cp.ki           = edu_drive->get_parameter(controllerID + std::string(".ki")).as_double();
      cp.kd           = edu_drive->get_parameter(controllerID + std::string(".kd")).as_double();
      cp.antiWindup   = edu_drive->get_parameter(controllerID + std::string(".antiWindup")).as_int();

      cp.responseMode   = (responseMode==0 ? edu::CAN_RESPONSE_RPM : edu::CAN_RESPONSE_POS);

      // --- Motor parameters ---------
      for(int d=0; d<2; d++)
      {
         std::string driveID = controllerID + std::string(".drive") + std::to_string(d);
         edu_drive->declare_parameter(driveID + std::string(".channel"), 0);
         edu_drive->declare_parameter<std::vector<double>>(driveID + std::string(".kinematics"), std::vector<double>{0.0,0.0,0.0});

         cp.motorParams[d].channel = edu_drive->get_parameter(driveID + std::string(".channel")).as_int();
         cp.motorParams[d].kinematics = edu_drive->get_parameter(driveID + std::string(".kinematics")).as_double_array();
      }
      // ------------------------------

      controllerParams.push_back(cp);
   }
   // -------------------------

   edu::SocketCAN can(canInterface);
   can.startListener();
   
   RCLCPP_INFO_STREAM(edu_drive->get_logger(), "CAN Interface: " << canInterface);

   bool verbosity;
   edu_drive->declare_parameter("verbosity", false);
   verbosity = edu_drive->get_parameter("verbosity").as_bool();

   edu_drive->initDrive(controllerParams, can, usingPwrMgmt, verbosity);
   edu_drive->run();

   can.stopListener();
}
