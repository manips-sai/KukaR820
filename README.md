# KUKA R820 Driver
There are two drivers for controlling the robot: 
- Sunrise OS application running on the control cabinet to receive torque commands
- Client application running on host control computer to send desired torques to the robot and read robot states 

***

## Sunrise OS

The first step to setup the driver is to load the Sunrise OS Workbench in a Windows PC. The software can be installed using the CD that came with the robot. The CD will also contain the FRI (Fast Robotic Interface) drivers that are needed to be installed in the Sunrise OS Workbench. 

Add the FRITorqueDriver.java file as a Sunrise application in the current project. The java application (to be deployed in the control cabinet) will facilitate the 1 ms FRI cyclic communication between the cabinet and the host computer via ethernet on the KONI port. The program is setup to do a position holding primitive (with zero gains) with a torque overlay. This is done because KUKA FRI doesn't have a direct torque control primitive application. 

The process to upload a new application to the cabinet is to connect the Windows PC via ethernet to port X66 on the cabinet. Configure the PC ethernet to IP address of 172.31.1.150 (netmask /24) and Gateway of 10.2.1.2.  The X66 port has IP of 172.31.1.147 (netmask /16). After, start the Sunrise OS Workbench software and load the project. You can add a new application by right-clicking on "application" in the left menu and adding a Sunrise application using a template, or you can directly move a .java application file over to the directory. 
	
The safety configuration can be adjusted in the safety.sconf tab in the left menu. Right now, the first 3 entries are unchecked, which aren't needed for research application. These need to be disabled in order to enable robot motion. 

The station setup can be adjusted in the StationSetup.cat tab. This is mainly used to enable the FRI software (in the Software tab) and to specify the MF end-effector configuration (which is already setup).

If you want to make changes, then the procedure is to first go to the "Install" tab in the StationSetup.cat, and click on it. This will show a pop-up displaying the IP of the X66 port highlighted in green. Follow the instructions to install the station setup, and then wait for the controller cabinet to reset (it will reset after a while). When the SmartPAD is booted up and goes into the main page, then click the sync button on the Workbench software (5th icon from the left on the top bar). This will load all applications into the controller cabinet.

***

## Linux driver

The host linux computer is configured with IP 192.170.10.1 (netmask /24, or 255.255.255.0). The KONI port has the IP 192.170.10.3/24. Note that the KONI port IP has been changed to work with a dual arm setup. 

To build the driver, first build the hiredis and jsconcpp libraries in the /lib folder. 
- For hiredis, simply sudo make, then sudo make install
- For jsconcpp, make a /build directory, go into the build directory, and sudo cmake .. && make -j8

Afterwards, make the KUKA FRI drivers. 
- Go into the /build/GNUMake folder, and sudo make -j8 for 3 times sequentially

There should be a LBRTorqueOverlay program in the bin folder.

*** 
## Operation

The driver works by reading the updated joint position from the robot, publishing robot states (q, dq) to the local redis server, and reading desired torques from the local redis server. To run a controller, make sure that the program is reading/writing from the following keys:
- sai2::iiwa14::sensors::q
- sai2::iiwa14::sensors::dq
- sai2::iiwa14::actuators::fgc

The controller should be running at 1 kHz, which is the same speed as the KUKA FRI connection (1 ms). Before running the driver, make sure that you ran the Positioning and GMS Referencing application on the SmartPAD. The order of operation is:
1. Start the FRITorque application on the SmartPAD, which initiates the Java application on the cabinet. The program will wait for a connection before starting the torque control routine. The robot will still be locked after running the application.
2. Start the redis server in a terminal. The recommendation is to run taskset --cpu-list 7 redis-server, which starts the redis server on the 8th core. The computer is currently setup to isolate the last 3 cores, so no process will be running on them unless explicitly instructed.
3. Start the LBRTorqueOverlay program from the /bin folder of the drivers with taskset --cpu-list 6 ./LBRTorqueOverlay. This will show controller state outputs in the terminal. Make sure that the joint values are read and the robot is in state 4 (commanding). You should check the redis keys as well.
4. Start the SAI controller with taskset --cpu-list 5 ./controller.
5. Stop the operation by hitting the e-stop on the SmartPAD, then terminating the the controller and LBRTorqueOverlay programs.
6. Restart with the same order of operation.

**Note that the KUKA FRI controller already does the gravity and centrifugal/coriolis compensation already.** Thus, the torque command being sent shouldn't include those compensations. If needed, some joint space velocity damping can be added in either the controller or in the Sunrise application. 
