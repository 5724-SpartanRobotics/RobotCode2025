// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.Elastic;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private RobotContainer _RobotContainer;

    private boolean __IsFirstConnection = true;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        if (!isDebug()) {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        _RobotContainer = new RobotContainer();
        _RobotContainer.robotFinishedBooting();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        NetworkTableInstance.getDefault().getEntry("/Match Time").setDouble(DriverStation.getMatchTime());
        NetworkTableInstance.getDefault().getEntry("/Voltage").setDouble(RobotController.getBatteryVoltage());
        if (DriverStation.isDSAttached() && __IsFirstConnection) {
            Elastic.selectTab("Autonomous");
            __IsFirstConnection = false;
        }
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        System.out.println("=== !!! Autonomous Started !!! ===");
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() { }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        _RobotContainer.ElevatorToStartingHeight();
        Elastic.selectTab("Real Teleoperated");
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        _RobotContainer.ResetPidReferences();
        _RobotContainer.StopPidRamps();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    /** Gets the current alliance color from DS and returns a WPILib color (red or blue) */
    public static edu.wpi.first.wpilibj.util.Color allianceToColor() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ?
        edu.wpi.first.wpilibj.util.Color.kBlue : edu.wpi.first.wpilibj.util.Color.kRed;
    }

    public static boolean isDebug() {
        // Code from ChatGPT. It sees if a debugger is attached by looking
        // at the arguements passed to the JVM when it was started.
        return java.lang.management.ManagementFactory.getRuntimeMXBean()
            .getInputArguments().toString().contains("-agentlib:jdwp");
    }
}
