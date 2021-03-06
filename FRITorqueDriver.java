package application;

// This file is loosely based on the KUKA sample code for the FRI interface.

// import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PositionHold;
// import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * Creates a FRI Session.
 */
public class FRITorqueDriver extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private int _clientPort;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.200";
        _clientPort = 30200;
    }

    private void doFRISession(FRIConfiguration friConfiguration) {
    	getLogger().info("Creating FRI connection to " + friConfiguration.getHostName() +
    			":" + friConfiguration.getPortOnRemote());
    	getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(3600, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");
        FRIJointOverlay jointOverlay = new FRIJointOverlay(
            friSession, ClientCommandMode.TORQUE);
        
        // Using standard implementation with zero gain 
        JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
        ctrMode.setStiffnessForAllJoints(0.0);
        ctrMode.setDampingForAllJoints(0.0);
        
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

        try {
          _lbr.move(posHold.addMotionOverlay(jointOverlay));
        } catch (final CommandInvalidException e) {
          getLogger().error(e.getLocalizedMessage());
        }

        // done
        friSession.close();
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(1);  // 1 ms controller sample time 
        friConfiguration.setReceiveMultiplier(1);  // 1:1 message receiving (1 ms)
        friConfiguration.setPortOnRemote(_clientPort);
        while (true) {
          doFRISession(friConfiguration);
        }
    }

    /**
     * main.
     *
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRITorqueDriver app = new FRITorqueDriver();
        app.runApplication();
    }

}
