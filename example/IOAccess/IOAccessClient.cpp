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



1.16
 */
#include <cstring>
#include <cstdio>

#include "IOAccessClient.h"
#include "friLBRState.h"
#include "friLBRCommand.h"

#include <iostream>

using namespace KUKA::FRI;

// Added
const std::string JOINT_ANGLES_KEY = "sai2::iiwa14::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwa14::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwa14::actuators::fgc";

//******************************************************************************
IOAccessClient::IOAccessClient() :
            analogMax(1), analogMin(0), digitalMax(60), digitalMin(0), step(0.1)
{
   printf("IOAccessClient initialized:\n");
   startTime = clock();
   HiredisServerInfo info;
   info.hostname_ = "127.0.0.1";
   info.port_ = 6379;
   info.timeout_ = { 1, 500000 }; // 1.5 seconds
   redis_client = new CDatabaseRedisClient();
   redis_client->serverIs(info);
   printf("Connected to redis server\n");
   redis_client->setCommandIs("sai2::kuka820::status", "on");
}

//******************************************************************************
IOAccessClient::~IOAccessClient()
{
}

//******************************************************************************
void IOAccessClient::onStateChange(KUKA::FRI::ESessionState newState, KUKA::FRI::ESessionState oldState)
{
}

void IOAccessClient::monitor()
{
   getAndSetExample();
   getAndSetRedisExample();
}

void IOAccessClient::waitForCommand()
{
   getAndSetExample();
   getAndSetRedisExample();
}

void IOAccessClient::command()
{
   getAndSetExample();
   getAndSetRedisExample();
}

void IOAccessClient::getAndSetExample()
{
  const int TICK_PER_SEC = 1000;

  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> diff = t_end - t_start;
  std::cout << std::scientific << diff.count() << std::endl;
  t_start = t_end;

   // LIMIT
   double tempAnalog = robotState().getAnalogIOValue("FRI.Out_Analog_Deci_Seconds");
   if (tempAnalog > analogMax)
   {
      robotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", analogMax);
   }
   else if (tempAnalog < analogMin)
   {
      robotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", analogMin);
   }

   long tempDigital = robotState().getDigitalIOValue("FRI.Out_Integer_Seconds");
   if (tempDigital > digitalMax)
   {
      robotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", digitalMax);
   }
   else if (tempDigital < digitalMin)
   {
      robotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", digitalMin);
   }

   bool isEnabled = robotState().getBooleanIOValue("FRI.In_Bool_Clock_Enabled");

   if (isEnabled)
   {
      double secDiff = (double)(clock() - startTime) / TICK_PER_SEC;

      if (secDiff >= 100)
      {
         double analogValue = robotState().getAnalogIOValue("FRI.Out_Analog_Deci_Seconds");
         analogValue = analogValue + step;
         if (analogValue < analogMax)
         {
            robotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", analogValue);
         }
         else
         {
            robotCommand().setAnalogIOValue("FRI.Out_Analog_Deci_Seconds", analogMin);

            long digitalValue = robotState().getDigitalIOValue("FRI.Out_Integer_Seconds") + 1;
            if (digitalValue < digitalMax)
            {
               robotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", digitalValue);
            }
            else
            {
               robotCommand().setDigitalIOValue("FRI.Out_Integer_Seconds", digitalMin);
            }
         }
         startTime = clock();
      }
   }
}

void IOAccessClient::getAndSetRedisExample()
{
   // get/set robot state values
   _time = robotState().getSampleTime();
   memcpy(_q1, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
   for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
   {
      _dq[i] = (_q1[i] - _q0[i]) / _time;  // finite difference joint velocities
      _q0[i] = _q1[i];  // update q0 position
   }
   redis_client->setDoubleArray(JOINT_ANGLES_KEY, _q1, 7);
   redis_client->setDoubleArray(JOINT_VELOCITIES_KEY, _dq, 7);
}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
