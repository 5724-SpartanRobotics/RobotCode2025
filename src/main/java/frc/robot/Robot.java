// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DebugLevel;
import frc.robot.lib.Elastic;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private static Robot _This;
    private boolean _IsFirstDsConnection = true;
    
    private final RobotContainer _RobotContainer;
    private final Timer _DisabledTimer;

    /**
         * This function is run when the robot is first started up and should be used for any
        * initialization code.
        */
    public Robot() {
        if (!Main.isDebug() && isReal()) {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        _This = this;
        _RobotContainer = new RobotContainer();
        _DisabledTimer = new Timer();

        _RobotContainer.robotFinishedBooting();
        if (isSimulation() || DebugLevel.isOrAll(DebugLevel.Autonomous)) { DriverStation.silenceJoystickConnectionWarning(true); }
        SmartDashboard.putString("Debug Mode", Constants.DebugTraceLevel.toString());
    }

    public static Robot getInstance() {
        return _This;
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
        if (_IsFirstDsConnection && DriverStation.isDSAttached()) {
            _IsFirstDsConnection = false;
            Elastic.selectTab("Autonomous");
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
        _RobotContainer.setMotorBrake(true);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        _RobotContainer.setMotorBrake(true);
        _DisabledTimer.reset();
        _DisabledTimer.start();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        if (_DisabledTimer.hasElapsed(Constants.Robot.WheelLockTime)) {
            _RobotContainer.setMotorBrake(false);
            _DisabledTimer.stop();
            _DisabledTimer.reset();
        }
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
