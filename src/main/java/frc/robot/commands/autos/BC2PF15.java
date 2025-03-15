package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.Constants.TimeDuration;
import frc.robot.commands.ArmRotateToSetpointCommand;
import frc.robot.commands.ClawRunForDurationCommand;
import frc.robot.commands.ElevatorToSetpointCommand;
import frc.robot.commands.WristRotateToSetpointCommand;
import frc.robot.commands.ClawRunForDurationCommand.ClawRunMode;
import frc.robot.subsystems.ArmSubSys;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubSys;

/**
 * Auto: <b>B C 2P F1,5</b>
 * <ul>
 * <li>Alliance: Blue</li>
 * <li>Starting Position: Center</li>
 * <li>Game Pieces scored: 2</li>
 * <li>Scored on Reef Faces: 1 & 5 (clockwise from furthest center face)</li>
 * </ul>
 */
public class BC2PF15 extends SequentialCommandGroup {
    public BC2PF15(
        AutoFactory autoFactory,
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubSys armSubSystem,
        ClawSubsystem clawSubsystem,
        WristSubSys wristSubsystem
    ) {
        addCommands(
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Intake, TimeDuration.Instant), // Initial intake
            Commands.parallel( // First run to face 1
                autoFactory.resetOdometry("B C 2P F1,5 1-3"), // Reset odometry & run the robot to face 1
                new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn), // Set the elevator to setpoint while moving
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn) // Move the arm while moving
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Outtake, TimeDuration.Instant), // place the coral on face 1 L4
            Commands.parallel( // Run to hp for coral intake
                autoFactory.resetOdometry("B C 2P F1,5 2-3"),
                new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn), // move the elevator
                new WristRotateToSetpointCommand(wristSubsystem, ElevatorAndArmConstants.WristMax), // move the wrist
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateCoralPosn) // rotate the arm
            ),
            new WaitCommand(2), // wait 2s for successful intake
            Commands.parallel( // move to face 5
                autoFactory.resetOdometry("B C 2P F1,5 3-3"), // move the robot over to face 5
                new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn),
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn)
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Outtake, TimeDuration.Instant), // place coral on face 5 L4
            new RunCommand(() -> {})
        );
    }
}
