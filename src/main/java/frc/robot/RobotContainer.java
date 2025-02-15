package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    public LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private AlgaeSubsystem _AlgaeSubSys;
    private CommandJoystick _DriverController;
    private CommandXboxController _OperatorController;

    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys(_LedSubsystem);
        _AlgaeSubSys = new AlgaeSubsystem();
        _DriverController = new CommandJoystick(0);
        _OperatorController = new CommandXboxController(1);

        _TeleopSwerve = new TeleopSwerve(_DriveTrainSubsystem, _DriverController);

        _DriveTrainSubsystem.setDefaultCommand(_TeleopSwerve);

        configureBindings();
    }

    private void configureBindings()
    {
        _DriverController.button(7).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.zeroGyro();
        }));
       //algae controls
        _OperatorController.a().onTrue(new InstantCommand(() -> {
            _AlgaeSubSys.RotateToOut();
        }, _AlgaeSubSys));
        _OperatorController.y().onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.IncrementSmallDegrees();
        }, _AlgaeSubSys));
        _OperatorController.x().onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.DecrementSmallDegrees();
        }, _AlgaeSubSys));

        //elevator up and down
        _OperatorController.rightBumper().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementElevatorUp();
        }, _ElevatorAndArmSubSys));
        _OperatorController.leftBumper().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementElevatorUp();
        }, _ElevatorAndArmSubSys));
        //arm rotate
        _OperatorController.povUp().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmRotate();
        }, _ElevatorAndArmSubSys));
        _OperatorController.povDown().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmRotate();
        }, _ElevatorAndArmSubSys));
        //arm extend
        _OperatorController.povRight().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.IncrementArmExtend();
        }, _ElevatorAndArmSubSys));
        _OperatorController.povLeft().onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.DecrementArmExtend();
        }, _ElevatorAndArmSubSys));
        //claw run - Right joystick Y axis is number 5. Forward yeilds a negative input, reverse a positive
        _OperatorController.axisLessThan(5, -0.4).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0.3);
        }, _ElevatorAndArmSubSys));
        _ElevatorAndArmSubSys.setDefaultCommand(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(0);
        }, _ElevatorAndArmSubSys));
        _OperatorController.axisGreaterThan(5, 0.4).onTrue(new InstantCommand(() ->{
            _ElevatorAndArmSubSys.ClawRun(-0.3);
        }, _ElevatorAndArmSubSys));
    }
}
