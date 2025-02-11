package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ElevatorAndArmSubSys;

public class RobotContainer {
private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
private CommandXboxController _OperatorController;

public RobotContainer(){
    _ElevatorAndArmSubSys = new ElevatorAndArmSubSys();
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
