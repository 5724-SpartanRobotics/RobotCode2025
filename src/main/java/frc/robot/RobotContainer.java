package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.PresetCommands;
import frc.robot.lib.Elastic;
import frc.robot.lib.XyPair;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer extends SubsystemBase {
    public final CommandJoystick _DriverController = new CommandJoystick(0);
    public final CommandJoystick _OperatorController = new CommandJoystick(1);
    public final AutoChooser _AutoChooser;

    private final DriveTrainSubsystem _DriveTrainSubsystem;
    private final LedSubsystem _LedSubsystem;
    private final AlgaeSubsystem _AlgaeSubsystem;
    private final ArmSubsystem _ArmSubsystem;
    private final ClawSubsystem _ClawSubsystem;
    private final ElevatorSubsystem _ElevatorSubsystem;
    private final WristSubsystem _WristSubsystem;

    protected Command driveFieldOrientedDirectAngle;
    protected Command driveFieldOrientedAnglularVelocity;
    protected Command driveRobotOrientedAngularVelocity ;
    protected Command driveSetpointGen;
    protected Command driveFieldOrientedDirectAngleKeyboard;
    protected Command driveFieldOrientedAnglularVelocityKeyboard;
    protected Command driveSetpointGenKeyboard;
    
    public RobotContainer() {
        super();
        _DriveTrainSubsystem = new DriveTrainSubsystem("swerve/vortex");
        _LedSubsystem = new LedSubsystem();
        _AlgaeSubsystem = new AlgaeSubsystem();
        _ArmSubsystem = new ArmSubsystem();
        _ClawSubsystem = new ClawSubsystem(_LedSubsystem);
        _ElevatorSubsystem = new ElevatorSubsystem();
        _WristSubsystem = new WristSubsystem();

        PresetCommands.initialize(_ElevatorSubsystem, _ArmSubsystem, _WristSubsystem);
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        
        _AutoChooser = new AutoChooser();
        
        configureDriveCommands();
        configureBindings();
        configureAutos();
        configureActions();
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

        configureSimBindings();
        if (DriverStation.isTest()) {
            configureTestBindings();
            return;
        } 

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


        _OperatorController.button(Constants.Controller.OperatorMap.AlgaeManualIncrement)
            .whileTrue(_AlgaeSubsystem.rotateAtSpeedCmd(Constants.Algae.Speeds.Increment))
            .onFalse(_AlgaeSubsystem.zeroSpeedCmd());
        _OperatorController.button(Constants.Controller.OperatorMap.AlgaeManualDecrement)
            .whileTrue(_AlgaeSubsystem.rotateAtSpeedCmd(Constants.Algae.Speeds.Decrement))
            .onFalse(_AlgaeSubsystem.zeroSpeedCmd());
        _OperatorController.button(Constants.Controller.OperatorMap.AlgaeSetpointOut)
            .onTrue(_AlgaeSubsystem.toSetpoint(AlgaeSubsystem.RotatePosition.Out));
        _OperatorController.button(Constants.Controller.OperatorMap.AlgaeSetpointIn)
            .onTrue(_AlgaeSubsystem.toSetpoint(AlgaeSubsystem.RotatePosition.In));
        _OperatorController.button(Constants.Controller.OperatorMap.AlgaeSetpointHold)
            .onTrue(_AlgaeSubsystem.toSetpoint(AlgaeSubsystem.RotatePosition.Hold));

        _OperatorController.povUp().onTrue(_ElevatorSubsystem.incrementPositionCmd());
        _OperatorController.povDown().onTrue(_ElevatorSubsystem.decrementPositionCmd());

        _OperatorController.axisLessThan(Constants.Controller.OperatorMap.ArmRotateAxis, Constants.Controller.OperatorMap.ArmRotateIncrementThreshold)
            .whileTrue(_ArmSubsystem.incrementPositionCmd());
        _OperatorController.axisGreaterThan(Constants.Controller.OperatorMap.ArmRotateAxis, Constants.Controller.OperatorMap.ArmRotateDecrementThreshold)
            .whileTrue(_ArmSubsystem.decrementPositionCmd());

        _OperatorController.button(Constants.Controller.OperatorMap.ClawExpel)
            .onTrue(_ClawSubsystem.runCmd(ClawSubsystem.IntakeMode.Expel))
            .onFalse(_ClawSubsystem.runCmd(ClawSubsystem.IntakeMode.Stop));
        _OperatorController.button(Constants.Controller.OperatorMap.ClawIntake)
            .onTrue(_ClawSubsystem.runCmd(ClawSubsystem.IntakeMode.Intake))
            .onFalse(_ClawSubsystem.runCmd(ClawSubsystem.IntakeMode.Stop));

        _OperatorController.button(Constants.Controller.OperatorMap.PresetHome).onTrue(PresetCommands.Home());
        _OperatorController.button(Constants.Controller.OperatorMap.PresetIntake).onTrue(PresetCommands.Intake());
        _OperatorController.button(Constants.Controller.OperatorMap.PresetL4).onTrue(PresetCommands.L4());
        _OperatorController.button(Constants.Controller.OperatorMap.PresetL3).onTrue(PresetCommands.L3());
        _OperatorController.button(Constants.Controller.OperatorMap.PresetL2).onTrue(PresetCommands.L2());
    }

    private void configureSimBindings() {
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
    }

    private void configureTestBindings() {
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
        }
    }

    private void configureAutos() {
        // Add autos here.

        SmartDashboard.putData(_AutoChooser);
        RobotModeTriggers.autonomous().whileTrue(_AutoChooser.selectedCommandScheduler());
    }

    private void configureActions() {
        RobotModeTriggers.autonomous().onTrue(Elastic.sendNotificationAndPrintCommand(
            new Elastic.Notification(
                Elastic.Notification.NotificationLevel.INFO,
                "Robot Mode Changed", "The robot is now in autonomous mode!"
            )
        ));

        RobotModeTriggers.teleop()
            .onTrue(
                new InstantCommand(() -> {
                    Elastic.selectTab("Real Teleoperated");
                    _ElevatorSubsystem.to(Units.Inches.of(1.0));
                }, _ElevatorSubsystem)
            )
            .onFalse(killRampsCmd());

        RobotModeTriggers.disabled().onTrue(killRampsCmd());
    }

    public void setMotorBrake(boolean brake) {
        _DriveTrainSubsystem.setMotorBrake(brake);
    }

    public void robotFinishedBooting() {
        _LedSubsystem.setColorForDurationCmd(LedSubsystem.kDefaultNotificationColor, Units.Seconds.of(2)).schedule();
        _DriveTrainSubsystem.zeroGyro();
    }

    public void killRamps() {
        _AlgaeSubsystem.stop();
        _ArmSubsystem.stop();
        _ElevatorSubsystem.stop();
        _WristSubsystem.stop();

        _AlgaeSubsystem.resetReferences();
        _ArmSubsystem.resetReferences();
        _ElevatorSubsystem.resetReferences();
        _WristSubsystem.resetReferences();

        Elastic.sendNotification(new Elastic.Notification(
            Elastic.Notification.NotificationLevel.INFO, "Ramps reset",
            "All PidRamps have been stopped and reset."
        ));
    }

    public Command killRampsCmd() {
        return new InstantCommand(() -> {
            killRamps();
        }, _AlgaeSubsystem, _ArmSubsystem, _ElevatorSubsystem, _WristSubsystem);
    }
}
