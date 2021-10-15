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
const double DEG2RAD = 3.1415926535897 / 180;
const double max_torques[7] = {320 * 0.9, 320 * 0.9, 176 * 0.9, 176 * 0.9, 110 * 0.9, 40 * 0.9, 40 * 0.9};  // ~90% torque limits
const double max_angles[7] = {170 * 0.9 * DEG2RAD, 120 * 0.9 * DEG2RAD, 170 * 0.9 * DEG2RAD, 
                                120 * 0.9 * DEG2RAD, 170 * 0.9 * DEG2RAD, 120 * 0.9 * DEG2RAD, 175 * 0.9 * DEG2RAD};
const double max_velocities[7] = {85 * 0.9 * DEG2RAD, 85 * 0.9 * DEG2RAD, 100 * 0.9 * DEG2RAD, 75 * 0.9 * DEG2RAD, 
                                  130 * 0.9 * DEG2RAD, 135 * 0.9 * DEG2RAD, 135 * 0.9 * DEG2RAD};
const double torque_offset[7] = {0, 0, 0, 0, 0, 0, 0};

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

   // check that controller is not running
   usleep(100000);
   redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, _torques, 7);  
   for(int i=0; i < 7; i++)
   {
    if(_torques[i] != 0)
    {
      std::cout << "ERROR : Stop the controller before running the driver\n" << std::endl;
      exit(0);
    }
  }

   // Reset values
   double zero_array[7] = {};
   redis_client->setDoubleArray(JOINT_ANGLES_KEY, zero_array, 7);
   redis_client->setDoubleArray(JOINT_VELOCITIES_KEY, zero_array, 7);
   redis_client->setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, zero_array, 7);
   redis_client->setCommandIs("sai2::kuka820::driver::status", "on");
   for (int i = 0; i < 7; i++){ _torques[i] = 0.0; }
   for (int i = 0; i < 10; i++)
   {
    for (int j = 0; j < 7; j++) 
    { 
      _dq_buffer[i][j] = 0.0; 
    }
   }
   _exit_counter = 0;
   _kv_exit = 10;  
   _buffer_ind = 0;
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

      // Check joint soft limits 
      for (int i = 0; i < 7; i++)
      {
        if (abs(_q1[i]) > max_angles[i])
        {
          std::cout << "Soft joint angle limits exceeded at joint " << i << std::endl;

          if (_exit_counter < 0)  // change to however long you want to decelerate for 
          {
            exit(0);
          }

          for (int j = 0; j < 7; j++)
          {
            _torques[j] = - _kv_exit * _dq[j];
          }

          _exit_counter--;

          break;
        }
        else if (abs(_dq[i]) > max_velocities[i])
        {
          std::cout << "Soft joint velocity limits exceeded at joint " << i << std::endl;

          if (_exit_counter < 0)  // change to however long you want to decelerate for 
          {
            exit(0);
          }

          for (int j = 0; j < 7; j++)
          {
            _torques[j] = - _kv_exit * _dq[j];
          }

          _exit_counter--;

          break;
        }
      }

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
   // timespec time;
   _time.tv_sec = robotState().getTimestampSec();
   _time.tv_nsec = robotState().getTimestampNanoSec();
   _elapsed_time = elapsedTime(_prev_time, _time);  // tested to be consistent 1 ms
   memcpy(_q1, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
   for (int i = 0; i < 7; i++)
   {
      _dq[i] = (_q1[i] - _q0[i]) / _elapsed_time;  // finite difference joint velocities
      _dq_buffer[_buffer_ind][i] = _dq[i];  // store velocity in the buffer
      _q0[i] = _q1[i];  // forward update
      _dq[i] = 0;  // reset for future updating 
   }

   // Update velocity with buffer average
   for (int i = 0; i < 7; i++)
   {
      for (int j = 0; j < 10; j++)
      {
        _dq[i] += 0.1 * _dq_buffer[j][i];
      }
   }

   _buffer_ind++;
   if (_buffer_ind == 10) {
    _buffer_ind = 0;
   } 

   // update previous time
   _prev_time = _time;

   // Set values
   redis_client->setDoubleArray(JOINT_ANGLES_KEY, _q1, 7);
   redis_client->setDoubleArray(JOINT_VELOCITIES_KEY, _dq, 7);

   // Get torques and clamp values
   redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, _torques, 7);

   // Clip torques
   for (int i = 0; i < 7; i++)
   {
     _torques[i] += torque_offset[i];
     if (_torques[i] < - max_torques[i])
     {
       _torques[i] = - max_torques[i];
     }
     else if (_torques[i] > max_torques[i])
     {
       _torques[i] = max_torques[i];
     }
   }
}

double LBRTorqueOverlayClient::elapsedTime(timespec start, timespec end)
{
	return (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec);
}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
