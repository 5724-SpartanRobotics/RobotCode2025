package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    public LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private CommandJoystick _DriverController;
    private CommandXboxController _OperatorController;

    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys(_LedSubsystem);
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
        _OperatorController.a().onTrue(new InstantCommand(() -> {
            _ElevatorAndArmSubSys.MoveToPickupPieceInRobot();
        }, _ElevatorAndArmSubSys));
    }
}
