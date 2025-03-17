package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.Constants.TimeDuration;
import frc.robot.commands.ClawRunForDurationCommand;
import frc.robot.commands.ClawRunForDurationCommand.ClawRunMode;
import frc.robot.commands.SetpointCommands.ArmRotateToSetpointCommand;
import frc.robot.commands.SetpointCommands.ElevatorToSetpointCommand;
import frc.robot.commands.SetpointCommands.WristRotateToSetpointCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Auto: <b>B C 2P F1,5</b>
 * <ul>
 * <li>Alliance: Blue</li>
 * <li>Starting Position: Center</li>
 * <li>Game Pieces scored: 2</li>
 * <li>Scored on Reef Faces: 1 & 5 (clockwise from furthest center face)</li>
 * </ul>
 */
public class BC2PF15 extends Command {
    private final AutoFactory _autoFactory;
    private final DriveTrainSubsystem _driveTrainSubsystem;
    private final ElevatorSubsystem _elevatorSubsystem;
    private final ArmSubsystem _armSubSystem;
    private final ClawSubsystem _clawSubsystem;
    private final WristSubsystem _wristSubsystem;
    
    public BC2PF15(
        AutoFactory autoFactory,
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubSystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem
    ) {
        _autoFactory = autoFactory;
        _driveTrainSubsystem = driveTrainSubsystem;
        _elevatorSubsystem = elevatorSubsystem;
        _armSubSystem = armSubSystem;
        _clawSubsystem = clawSubsystem;
        _wristSubsystem = wristSubsystem;
    }

    @Override
    public void execute() {
        Commands.sequence(
            new ClawRunForDurationCommand(_clawSubsystem, ClawRunMode.Intake, 0.5), // Initial intake
            Commands.parallel( // First run to face 1
                _autoFactory.resetOdometry("B C 2P F1,5 1-3"), // Reset odometry & run the robot to face 1
                new ElevatorToSetpointCommand(_elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn), // Set the elevator to setpoint while moving
                new ArmRotateToSetpointCommand(_armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn) // Move the arm while moving
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(_clawSubsystem, ClawRunMode.Outtake, TimeDuration.Instant), // place the coral on face 1 L4
            Commands.parallel( // Run to hp for coral intake
                _autoFactory.resetOdometry("B C 2P F1,5 2-3"),
                new ElevatorToSetpointCommand(_elevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn), // move the elevator
                new WristRotateToSetpointCommand(_wristSubsystem, ElevatorAndArmConstants.WristMax), // move the wrist
                new ArmRotateToSetpointCommand(_armSubSystem, ElevatorAndArmConstants.ArmRotateCoralPosn) // rotate the arm
            ),
            new WaitCommand(2), // wait 2s for successful intake
            Commands.parallel( // move to face 5
                _autoFactory.resetOdometry("B C 2P F1,5 3-3"), // move the robot over to face 5
                new ElevatorToSetpointCommand(_elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn),
                new ArmRotateToSetpointCommand(_armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn)
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(_clawSubsystem, ClawRunMode.Outtake, TimeDuration.Instant), // place coral on face 5 L4
            new RunCommand(() -> {})
        ).schedule();
    }
}
