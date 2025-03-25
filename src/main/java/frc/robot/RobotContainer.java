package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.lib.XyPair;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RobotContainer extends SubsystemBase {
    public final CommandJoystick _DriverController = new CommandJoystick(0);
    public final CommandJoystick OperatorController = new CommandJoystick(1);

    private final DriveTrainSubsystem _DriveTrainSubsystem;

    public Command driveFieldOrientedDirectAngle;
    public Command driveFieldOrientedAnglularVelocity;
    public Command driveRobotOrientedAngularVelocity ;
    public Command driveSetpointGen;
    public Command driveFieldOrientedDirectAngleKeyboard;
    public Command driveFieldOrientedAnglularVelocityKeyboard;
    public Command driveSetpointGenKeyboard;
    
    public RobotContainer() {
        super();
        _DriveTrainSubsystem = new DriveTrainSubsystem("swerve/vortex");
        
        configureDriveCommands();
        configureBindings();
    }

    @Override
    public void periodic() {
        super.periodic();
        NetworkTableInstance.getDefault().getEntry("/Match Time").setDouble(DriverStation.getMatchTime());
        NetworkTableInstance.getDefault().getEntry("/Voltage").setDouble(RobotController.getBatteryVoltage());
    }

    private XyPair __joystick__getTxXyPair() {
        return new XyPair(_DriverController.getX(), _DriverController.getY());
    }

    private XyPair __joystick__getRotXyPair() {
        return new XyPair(_DriverController.getTwist(), 0);
    }

    private void configureDriveCommands() {
        this.driveFieldOrientedDirectAngle = _DriveTrainSubsystem.driveFieldOriented(
            DriveTrainSubsystem.driveDirectAngle(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, this::__joystick__getRotXyPair)
        );
        this.driveFieldOrientedAnglularVelocity = _DriveTrainSubsystem.driveFieldOriented(
            DriveTrainSubsystem.driveAngularVelocity(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, this::__joystick__getRotXyPair)
        );
        this.driveRobotOrientedAngularVelocity  = _DriveTrainSubsystem.driveFieldOriented(
            DriveTrainSubsystem.driveRobotOriented(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, this::__joystick__getRotXyPair)
        );
        this.driveSetpointGen = _DriveTrainSubsystem.driveWithSetpointGeneratorFieldRelative(
            DriveTrainSubsystem.driveDirectAngle(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, this::__joystick__getRotXyPair)
        );
        this.driveFieldOrientedDirectAngleKeyboard = _DriveTrainSubsystem.driveFieldOriented(
            DriveTrainSubsystem.driveDirectAngleKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
        );
        this.driveFieldOrientedAnglularVelocityKeyboard = _DriveTrainSubsystem.driveFieldOriented(
            DriveTrainSubsystem.driveAngularVelocityKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
        );
        this.driveSetpointGenKeyboard = _DriveTrainSubsystem.driveWithSetpointGeneratorFieldRelative(
            DriveTrainSubsystem.driveDirectAngleKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
        );
    }

    private void configureBindings() {
        if (RobotBase.isSimulation()) _DriveTrainSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        else _DriveTrainSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        if (RobotBase.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1 ,4), Rotation2d.kCW_90deg);
            DriveTrainSubsystem.driveDirectAngleKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
                .driveToPose(
                    () -> target,
                    new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
                    new ProfiledPIDController(5, 0, 0, new Constraints(Constants.TwoPI, Math.PI))
                );
            
            _DriverController.button(Constants.Controller.DriverMap.ZeroGyro)
                .onTrue(Commands.runOnce(() -> _DriveTrainSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            _DriverController.button(Constants.Controller.DriverMap.SysIdDrive)
                .whileTrue(_DriveTrainSubsystem.sysIdDriveMotorCommand());
            _DriverController.button(Constants.Controller.DriverMap.DriveToPose)
                .whileTrue(Commands.runEnd(
                    () -> DriveTrainSubsystem.driveDirectAngleKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
                        .driveToPoseEnabled(true),
                    () -> DriveTrainSubsystem.driveDirectAngleKbd(_DriveTrainSubsystem.getSwerveDrive(), this::__joystick__getTxXyPair, () -> _DriverController.getTwist())
                        .driveToPoseEnabled(false)
                ));
        }
        if (DriverStation.isTest()) {
            _DriveTrainSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
            _DriverController.button(Constants.Controller.DriverMap.Lock_TEST)
                .whileTrue(Commands.runOnce(_DriveTrainSubsystem::lock, _DriveTrainSubsystem).repeatedly());
            _DriverController.button(Constants.Controller.DriverMap.DriveToDistance)
                .whileTrue(_DriveTrainSubsystem.driveToDistanceCommand(Units.Meters.of(1), Units.MetersPerSecond.of(0.2)));
            _DriverController.button(Constants.Controller.DriverMap.ZeroGyro)
                .onTrue(Commands.runOnce(() -> _DriveTrainSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            _DriverController.button(Constants.Controller.DriverMap.ZeroSwerveModules)
                .whileTrue(_DriveTrainSubsystem.centerModulesCommand());
        } else {
            _DriverController.button(Constants.Controller.DriverMap.DriveToDistance)
                .whileTrue(_DriveTrainSubsystem.driveToDistanceCommand(Units.Meters.of(1), Units.MetersPerSecond.of(0.2)));
            _DriverController.button(Constants.Controller.DriverMap.AddFakeVisionReading)
                .onTrue(Commands.runOnce(_DriveTrainSubsystem::addFakeVisionReading, _DriveTrainSubsystem));
            _DriverController.button(Constants.Controller.DriverMap.ZeroGyro)
                .onTrue(Commands.runOnce(_DriveTrainSubsystem::zeroGyro));
            _DriverController.button(Constants.Controller.DriverMap.ZeroSwerveModules)
                .whileTrue(_DriveTrainSubsystem.centerModulesCommand());
            _DriverController.button(Constants.Controller.DriverMap.Lock)
                .whileTrue(Commands.runOnce(_DriveTrainSubsystem::lock, _DriveTrainSubsystem).repeatedly());
        }
    }

    public Command getAutonomousCommand() {
        return _DriveTrainSubsystem.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake) {
        _DriveTrainSubsystem.setMotorBrake(brake);
    }
}
