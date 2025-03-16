package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.SetpointCommands.ArmRotateToSetpointCommand;
import frc.robot.commands.SetpointCommands.ElevatorToSetpointCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Auto: <b>B R 2P F2,3</b>
 * <ul>
 * <li>Alliance: Blue</li>
 * <li>Starting Position: Right</li>
 * <li>Game Pieces scored: 2</li>
 * <li>Scored on Reef Faces: 2 & 3 (clockwise from furthest center face)</li>
 * </ul>
 */
public class BR2PF23 extends SequentialCommandGroup {
    public BR2PF23(
        AutoFactory autoFactory,
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubSystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem
    ) {
        addCommands(
            Commands.sequence(
                Commands.parallel(
                    autoFactory.resetOdometry("R L 2P F2,3 1-3"), // Reset odometry & run the robot
                    new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn) // Set the elevator to setpoint while moving
                ),
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn), // Move the arm while stopped
                Commands.sequence(
                    Commands.deadline(new WaitCommand(0.5), new RunCommand(() -> {clawSubsystem.ClawRun(0.3);}, clawSubsystem)),
                    new RunCommand(() -> {clawSubsystem.ClawRun(0.0);}, clawSubsystem)
                ),
                new ArmRotateToSetpointCommand(armSubSystem, 0.0),
                Commands.parallel(
                    autoFactory.resetOdometry("R L 2P F2,3 2-3"),
                    new ElevatorToSetpointCommand(elevatorSubsystem, 0.0)
                ),
                new WaitCommand(2),
                Commands.parallel(
                    autoFactory.resetOdometry("R L 2P F2,3 3-3"),
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
