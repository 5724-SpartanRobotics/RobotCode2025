package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.AprilTagLockonCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    private LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private AlgaeSubsystem _AlgaeSubSys;
    private VisionSubsystem _VisionSubSys;
    private CommandJoystick _DriverController;
    private CommandJoystick _OperatorController;
    public static Interference InterferenceHelper;

    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _VisionSubSys = new VisionSubsystem(_DriveTrainSubsystem);
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys(_LedSubsystem);
        _AlgaeSubSys = new AlgaeSubsystem();
        InterferenceHelper = new Interference(_AlgaeSubSys, _ElevatorAndArmSubSys);
        _DriverController = new CommandJoystick(0);
        _OperatorController = new CommandJoystick(1);

        _TeleopSwerve = new TeleopSwerve(_DriveTrainSubsystem, _DriverController);

        _DriveTrainSubsystem.setDefaultCommand(_TeleopSwerve);
        CameraServer.startAutomaticCapture(); // Start camera server on zeroth index video device

        configureBindings();
    }

    private void configureBindings()
    {
        _DriverController.button(7).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.zeroGyro();
        }));
        // _DriverController.button(11).whileTrue(new AprilTagLockonCommand(_DriveTrainSubsystem, _VisionSubSys));
       
        //algae controls
        _OperatorController.button(7).onTrue(new InstantCommand(() -> {
            _AlgaeSubSys.RotateToOut();
        }, _AlgaeSubSys));
        _OperatorController.button(8).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.IncrementSmallDegrees();
        }, _AlgaeSubSys));
        _OperatorController.button(10).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.DecrementSmallDegrees();
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
        _OperatorController.axisLessThan(1, -0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmRotate();
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(1, 0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmRotate();
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
            _ElevatorAndArmSubSys.ClawRun(0.3);
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(1).onFalse(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0);
        }, _ElevatorAndArmSubSys));
        // _ElevatorAndArmSubSys.setDefaultCommand(new InstantCommand(() ->{
        //     _ElevatorAndArmSubSys.ClawRun(0);
        // }, _ElevatorAndArmSubSys));
        _OperatorController.button(2).onFalse(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0);
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(2).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(-0.3);
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(5).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.MoveToL4();
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.MoveToL3();
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(6).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.MoveToL2();
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(4).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.MoveToL1();
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(12).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.MoveToPickupPieceInRobot();
        }, _ElevatorAndArmSubSys));
    }

    public void robotFinishedBooting() {
        // new ParallelDeadlineGroup(new WaitCommand(2), new InstantCommand(() -> {_LedSubsystem.setColor(Color.kGray);}))
        //     .andThen(new InstantCommand(() -> {_LedSubsystem.reset();}))
        //     .execute();
    }
}
