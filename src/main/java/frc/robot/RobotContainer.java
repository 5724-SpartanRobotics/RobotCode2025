package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorAndArmSubSys;
import frc.robot.subsystems.LedSubsystem;

public class RobotContainer {
    private LedSubsystem _LedSubsystem;
    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private CommandXboxController _OperatorController;

    public RobotContainer(){
        _LedSubsystem = new LedSubsystem();
        _ElevatorAndArmSubSys = new ElevatorAndArmSubSys(_LedSubsystem);
        _OperatorController = new CommandXboxController(1);
        configureBindings();
    }

    private void configureBindings()
    {
        _OperatorController.a().onTrue(new InstantCommand(() -> {
            _ElevatorAndArmSubSys.MoveToPickupPieceInRobot();
        }, _ElevatorAndArmSubSys));
    }
}
