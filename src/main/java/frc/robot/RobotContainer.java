package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ArmExtendToSetpointCommand;
import frc.robot.commands.ArmRotateToSetpointCommand;
import frc.robot.commands.ElevatorToSetpointCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.Leave;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    private LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private ClawSubsystem _ClawSubsystem;
    private AlgaeSubsystem _AlgaeSubSys;
    @SuppressWarnings("unused")
    private VisionSubsystem _VisionSubSys;
    private CommandJoystick _DriverController;
    private CommandJoystick _OperatorController;

    public static Interference InterferenceHelper;
    public final AutoFactory autoFactory;
    public AutoChooser autoChooser;
    public final SendableChooser<Command> m_autos = new SendableChooser<Command>();


    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _VisionSubSys = new VisionSubsystem(_DriveTrainSubsystem);
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys();
        _ClawSubsystem = new ClawSubsystem(_LedSubsystem);
        _AlgaeSubSys = new AlgaeSubsystem();
        InterferenceHelper = new Interference(_ElevatorAndArmSubSys);
        _DriverController = new CommandJoystick(0);
        _OperatorController = new CommandJoystick(1);
        autoFactory = new AutoFactory(
            _DriveTrainSubsystem::getPose, // A function that returns the current robot pose
            _DriveTrainSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            _DriveTrainSubsystem::followTrajectory, // The drive subsystem trajectory follower 
             true,// If alliance flipping should be enabled 
            _DriveTrainSubsystem
        );
        @SuppressWarnings("unused")
        AutoRoutine routine = autoFactory.newRoutine("Blueleaveleft");
        // autoChooser = new AutoChooser();

        

        _TeleopSwerve = new TeleopSwerve(_DriveTrainSubsystem, _DriverController);

        _DriveTrainSubsystem.setDefaultCommand(_TeleopSwerve);
        CameraServer.startAutomaticCapture(); // Start camera server on zeroth index video device
        CameraServer.startAutomaticCapture(); // Start camera server on first index video device (auto inc)

        _DriveTrainSubsystem.zeroGyro();
        _DriveTrainSubsystem.flipGyro();
       

        configureBindings();
        // autoChooser.addRoutine("Blueleaveleft", null);
        // autoChooser.addCmd("Basic 2s Leave", );
        // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        configureAutos();
    }

    private void configureAutos() {
        m_autos.setDefaultOption("Default (noting)", new Command() {});
        m_autos.addOption("Basic 2s Leave", new Leave(_DriveTrainSubsystem));

        SmartDashboard.putData("Auto choices", m_autos);
    }

    private void configureBindings()
    {
        _DriverController.button(7).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.zeroGyro();
        }));
        // _DriverController.button(11).whileTrue(new AprilTagLockonCommand(_DriveTrainSubsystem, _VisionSubSys));
       
        //algae controls
        _OperatorController.button(8).whileTrue(new RunCommand(() ->{
            _AlgaeSubSys.AlgaeRotateAtSpeed(0.3);
        }, _AlgaeSubSys));
        _OperatorController.button(8).onFalse(new InstantCommand(() -> {
            _AlgaeSubSys.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubSys));
        _OperatorController.button(10).whileTrue(new RunCommand(() ->{
            _AlgaeSubSys.AlgaeRotateAtSpeed(-0.3);
        }, _AlgaeSubSys));
        _OperatorController.button(10).onFalse(new InstantCommand(() -> {
            _AlgaeSubSys.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubSys));
        _OperatorController.button(7).onTrue(new InstantCommand(() -> {
            _AlgaeSubSys.RotateToOut();
        }, _AlgaeSubSys));
        _OperatorController.button(11).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.RotateToIn();
        }, _AlgaeSubSys));
        _OperatorController.button(9).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.RotateToHoldAlgaePosn();
        }, _AlgaeSubSys));

        //elevator increment up and down
        _OperatorController.povUp().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementElevatorUp();
        }, _ElevatorAndArmSubSys));
        _OperatorController.povDown().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementElevatorUp();
        }, _ElevatorAndArmSubSys));

        //arm rotate - main Y axis
        _OperatorController.axisLessThan(1, -0.3).whileTrue(new RunCommand(() ->{
            _ElevatorAndArmSubSys.ArmRotateToPositionMoreThanCurrent();
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(1, 0.3).whileTrue(new RunCommand(() ->{
            _ElevatorAndArmSubSys.ArmRotateToPositionLessThanCurrent();
        }, _ElevatorAndArmSubSys));

        //arm extend
        _OperatorController.povRight().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmExtend();
        }, _ElevatorAndArmSubSys));
        _OperatorController.povLeft().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmExtend();
        }, _ElevatorAndArmSubSys));

        //claw run 
        _OperatorController.button(1).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0.3);
        }, _ClawSubsystem));
        _OperatorController.button(1).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0);
        }, _ClawSubsystem));
        // _ElevatorAndArmSubSys.setDefaultCommand(new InstantCommand(() ->{
        //     _ElevatorAndArmSubSys.ClawRun(0);
        // }, _ElevatorAndArmSubSys));
        _OperatorController.button(2).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0);
        }, _ClawSubsystem));
        _OperatorController.button(2).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(-0.3);
        }, _ClawSubsystem));
        //sequencial commands
        SequentialCommandGroup l4CommandGroups = new SequentialCommandGroup(
            new ElevatorToSetpointCommand((_ElevatorAndArmSubSys), ElevatorAndArmConstants.ElevatorL4Posn)
            // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
            .andThen(new ArmRotateToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmRotateL4Posn))
            .andThen(new ArmExtendToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmExtendL4Posn)));
        SequentialCommandGroup l3CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorAndArmSubSys), ElevatorAndArmConstants.ElevatorL3Posn)
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .andThen(new ArmRotateToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmRotateL3Posn))
                .andThen(new ArmExtendToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmExtendL3Posn)));
        SequentialCommandGroup l2CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorAndArmSubSys), ElevatorAndArmConstants.ElevatorL2Posn)
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .andThen(new ArmRotateToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmRotateL2Posn))
                .andThen(new ArmExtendToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmExtendL2Posn)));
        SequentialCommandGroup l1CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorAndArmSubSys), ElevatorAndArmConstants.ElevatorL1Posn)
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .andThen(new ArmRotateToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmRotateL1Posn))
                .andThen(new ArmExtendToSetpointCommand(_ElevatorAndArmSubSys, ElevatorAndArmConstants.ArmExtendL1Posn)));
        SequentialCommandGroup returnHomeCommandGroup = new SequentialCommandGroup(
                    new ArmExtendToSetpointCommand((_ElevatorAndArmSubSys), 0.0)
                    // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                    .andThen(new ArmRotateToSetpointCommand(_ElevatorAndArmSubSys, 0.0))
                    .andThen(new ElevatorToSetpointCommand(_ElevatorAndArmSubSys, 0.0)));
        _OperatorController.button(5).onTrue(l4CommandGroups);
        _OperatorController.button(3).onTrue(l3CommandGroups);
        _OperatorController.button(6).onTrue(l2CommandGroups);
        _OperatorController.button(4).onTrue(l1CommandGroups);
        _OperatorController.button(12).onTrue(returnHomeCommandGroup);
    }

    public void robotFinishedBooting() {
        _LedSubsystem.setColorForDuration(LedSubsystem.kDefaultNotificationColor, 2);
    }

    /**
     * Calling stop on the PID controlled parts of a subsystem calls stop on their PID ramp
     * instances, which stops them from setting a reference to the SparkMax ClosedLoop position
     * control and makes their ramp outputs follow the current position feedback.
     */
    public void StopPidRamps()
    {
        _AlgaeSubSys.AlgaeStop();
        _ElevatorAndArmSubSys.ArmExtendStop();
        _ElevatorAndArmSubSys.ArmRotateStop();
        _ElevatorAndArmSubSys.ElevatorStop();
    }

    public AutoRoutine Blueleaveleft() {
        AutoRoutine routine = autoFactory.newRoutine("1");
    
        // Load the routine's trajectories
        AutoTrajectory BlueLeaveLeft = routine.trajectory("Blue Leave Left");
    
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                BlueLeaveLeft.resetOdometry(),
                BlueLeaveLeft.cmd()
            )
        );
    
        return routine;
    }

    public void ElevtorToStartingHeight()
    {
        _ElevatorAndArmSubSys.ElevatorToPosition(2);
    }
}
