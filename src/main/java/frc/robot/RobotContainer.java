package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    private LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private AlgaeSubsystem _AlgaeSubSys;
    private CommandJoystick _DriverController;
    private CommandJoystick _OperatorController;
    public static Interference InterferenceHelper;

    public RobotContainer(){
        InterferenceHelper = new Interference(_AlgaeSubSys, _ElevatorAndArmSubSys);
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys(_LedSubsystem);
        _AlgaeSubSys = new AlgaeSubsystem();
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
        _OperatorController.axisLessThan(5, -0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementElevatorUp();
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(5, 0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementElevatorUp();
        }, _ElevatorAndArmSubSys));
        //arm rotate - main Y axis
        _OperatorController.axisLessThan(2, -0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmRotate();
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(2, 0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmRotate();
        }, _ElevatorAndArmSubSys));
        //arm extend
        _OperatorController.axisLessThan(6, -0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmExtend();
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(6, 0.3).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmExtend();
        }, _ElevatorAndArmSubSys));
        //claw run 
        _OperatorController.button(1).whileTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0.3);
        }, _ElevatorAndArmSubSys));
        _ElevatorAndArmSubSys.setDefaultCommand(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0);
        }, _ElevatorAndArmSubSys));
        _OperatorController.button(2).whileTrue(new InstantCommand(() ->{
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
}
