package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ArmRotateToSetpointCommand;
import frc.robot.commands.ElevatorToSetpointCommand;
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
            Commands.sequence(
                Commands.parallel(
                    autoFactory.resetOdometry("B C 2P F1,5 1-3"), // Reset odometry & run the robot
                    new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn) // Set the elevator to setpoint while moving
                ),
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn), // Move the arm while stopped
                Commands.sequence(
                    Commands.deadline(new WaitCommand(0.5), new RunCommand(() -> {clawSubsystem.ClawRun(0.3);}, clawSubsystem)),
                    new RunCommand(() -> {clawSubsystem.ClawRun(0.0);}, clawSubsystem)
                ),
                new ArmRotateToSetpointCommand(armSubSystem, 0.0),
                Commands.parallel(
                    autoFactory.resetOdometry("B C 2P F1,5 2-3"),
                    new ElevatorToSetpointCommand(elevatorSubsystem, 0.0)
                ),
                new WaitCommand(2),
                Commands.parallel(
                    autoFactory.resetOdometry("B C 2P F1,5 3-3"),
                    new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn)
                ),
                new ArmRotateToSetpointCommand(armSubSystem, 0.0),
                Commands.sequence(
                    Commands.deadline(new WaitCommand(0.5), new RunCommand(() -> {clawSubsystem.ClawRun(0.3);}, clawSubsystem)),
                    new RunCommand(() -> {clawSubsystem.ClawRun(0.0);}, clawSubsystem)
                ),
                new RunCommand(() -> {})
            )
        );
    }
}
