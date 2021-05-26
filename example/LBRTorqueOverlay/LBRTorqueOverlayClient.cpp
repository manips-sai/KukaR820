/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2018
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only.
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.16}
*/
#include <cstring>
#include <cstdio>
// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <math.h>
#include "LBRTorqueOverlayClient.h"

using namespace KUKA::FRI;

const std::string JOINT_ANGLES_KEY = "sai2::iiwa14::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwa14::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwa14::actuators::fgc";
const double max_torques[7] = {310, 310, 170, 170, 100, 35, 35};  // ~90% torque limits

//******************************************************************************
LBRTorqueOverlayClient::LBRTorqueOverlayClient()
{
   printf("LBRTorqueOverlayClient initialized:\n");

   // Connect to redis server
   HiredisServerInfo info;
   info.hostname_ = "127.0.0.1";
   info.port_ = 6379;
   info.timeout_ = { 1, 500000 }; // 1.5 seconds
   redis_client = new CDatabaseRedisClient();
   redis_client->serverIs(info);
   printf("Connected to redis server\n");

   // Reset values
   double zero_array[7] = {};
   redis_client->setDoubleArray(JOINT_ANGLES_KEY, zero_array, 7);
   redis_client->setDoubleArray(JOINT_VELOCITIES_KEY, zero_array, 7);
   redis_client->setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, zero_array, 7);
   redis_client->setCommandIs("sai2::kuka820::driver::status", "on");
   for (int i = 0; i < 7; i++){ _torques[i] = 0.0; }
}

//******************************************************************************
LBRTorqueOverlayClient::~LBRTorqueOverlayClient()
{
}

//******************************************************************************
void LBRTorqueOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         for (int i = 0; i < 7; i++){ _torques[i] = 0.0; }
         std::cout << "Reading joints" << std::endl;
         memcpy(_q0, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
         memcpy(_q1, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
         for (int i = 0; i < 7; i++) { _dq[i] = 0.0; };
         break;
      }
      default:
      {
        for (int i = 0; i < 7; i++){ _torques[i] = 0.0; }
        std::cout << "Reading joints" << std::endl;
        memcpy(_q0, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
        memcpy(_q1, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
        for (int i = 0; i < 7; i++) { _dq[i] = 0.0; };
        break;
      }
   }
}
//******************************************************************************

void LBRTorqueOverlayClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();

   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is
   // only necessary, that some torque values are sent. The LBR does not take the
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      setRedisInfo();
      robotCommand().setJointPosition(_q1);
      robotCommand().setTorque(_torques);
   }
}
//******************************************************************************
void LBRTorqueOverlayClient::command()
{
    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();

    // Check for correct ClientCommandMode.
    if (robotState().getClientCommandMode() == TORQUE)
    {
      // start = std::clock();
      setRedisInfo();

      // duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
      // start = std::clock();
      // std::cout << "Redis info " << duration << std::endl;

      robotCommand().setJointPosition(_q1);

      // duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
      // start = std::clock();
      // std::cout << "Set joint position " << duration << std::endl;

      robotCommand().setTorque(_torques);

      // duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
      // start = std::clock();
      // std::cout << "Set joint torque  " << duration << std::endl;
    }
}

void LBRTorqueOverlayClient::setRedisInfo()
{
   // get/set robot state values
   //  _time = robotState().getSampleTime();  // tested to be consistent 1 ms
   timespec time;
   time.tv_sec = robotState().getTimestampSec();
   time.tv_nsec = robotState().getTimestampNanoSec();
   _time = elapsedTime(_prev_time, time);  // tested to be consistent 1 ms
   memcpy(_q1, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
   for (int i = 0; i < 7; i++)
   {
      _dq[i] = (_q1[i] - _q0[i]) / _time;  // finite difference joint velocities
      _q0[i] = _q1[i];  // forward update
   }

   // update previous time
   _prev_time = time;

   // Set values
   redis_client->setDoubleArray(JOINT_ANGLES_KEY, _q1, 7);
   redis_client->setDoubleArray(JOINT_VELOCITIES_KEY, _dq, 7);

   // Get torques and clamp values
   redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, _torques, 7);

   // Clip torques
   for (int i = 0; i < 7; i++)
   {
     if (_torques[i] < -max_torques[i])
     {
       _torques[i] = -max_torques[i];
     }
     else if (_torques[i] > max_torques[i])
     {
       _torques[i] = max_torques[i];
     }
   }
}

double LBRTorqueOverlayClient::elapsedTime(timespec start, timespec end)
{
	return (end.tv_sec - start.tv_sec) + 1e-9*(end.tv_nsec - start.tv_nsec);
}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
