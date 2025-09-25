package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ClawRunForDurationCommand;
import frc.robot.commands.ClawRunForDurationCommand.ClawRunMode;
import frc.robot.commands.PresetCommands;
import frc.robot.commands.SetpointCommands;
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
public class BC2PF15 extends SequentialCommandGroup {
    private final AutoFactory _autoFactory;
    
    public BC2PF15(
        AutoFactory autoFactory,
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubSystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem,
        PresetCommands presetCommands
    ) {
        _autoFactory = autoFactory;

        addCommands(Commands.sequence(
            new InstantCommand(() -> {driveTrainSubsystem.flipGyro();}),
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Intake, 0.5),
            Commands.parallel(
                Commands.sequence(
                    _autoFactory.resetOdometry("B C 2P F1,5 1-3"),
                    _autoFactory.trajectoryCmd("B C 2P F1,5 1-3"),
                    driveTrainSubsystem.brakeCmd()
                ),
                new SequentialCommandGroup(
                    new WaitCommand(0)
                    .alongWith(new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn))
                    .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
                    .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn))
                )
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Outtake, 0.5), // place the coral on face 1 L4
            Commands.parallel( // Run to hp for coral intake
                Commands.sequence(
                    _autoFactory.resetOdometry("B C 2P F1,5 2-3"),
                    _autoFactory.trajectoryCmd("B C 2P F1,5 2-3"),
                    driveTrainSubsystem.brakeCmd()
                ),
                new SequentialCommandGroup(
                    new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn)
                    .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, ElevatorAndArmConstants.WristMax))
                    .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateCoralPosn))
                )
            ),
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Intake, 2.0),
            Commands.parallel( // move to face 5
                Commands.sequence(
                    _autoFactory.resetOdometry("B C 2P F1,5 3-3"),
                    _autoFactory.trajectoryCmd("B C 2P F1,5 3-3"),
                    driveTrainSubsystem.brakeCmd()
                ), // move the robot over to face 5
                new SequentialCommandGroup(
                    new WaitCommand(0)
                    .alongWith(new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn))
                    .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
                    .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn))
                )
            ),
            new WaitCommand(0.5), // wait for the robot to settle
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Outtake, 0.5), // place coral on face 5 L4
            driveTrainSubsystem.brakeCmd(),
            new SequentialCommandGroup(
                new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0)
                .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubSystem, 0.0))
                .alongWith(new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, 0.0))
            ),
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Stopped, 15)
        ));
    }
}
