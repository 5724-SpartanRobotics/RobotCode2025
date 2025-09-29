package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class PresetCommands {
    private final ElevatorSubsystem _ElevatorSubsystem;
    private final WristSubsystem _WristSubsystem;
    private final ArmSubsystem _ArmSubsystem;
    private final LedSubsystem _LedSubsystem;

    public PresetCommands(
        ElevatorSubsystem elevatorSubsystem,
        WristSubsystem wristSubsystem,
        ArmSubsystem armSubsystem,
        LedSubsystem ledSubsystem
    ) {
        _ElevatorSubsystem = elevatorSubsystem;
        _WristSubsystem = wristSubsystem;
        _ArmSubsystem = armSubsystem;
        _LedSubsystem = ledSubsystem;
    }

    public Command L4() { return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> { _LedSubsystem.setColorForDurationNTimes(LedSubsystem.kDefaultNotificationColor, 0.5, 1); }),
            new WaitCommand(0)
            .alongWith(new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn))
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, ElevatorAndArmConstants.ArmRotateL4Posn))
    )); }
    public Command L3() { return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> { _LedSubsystem.setColorForDurationNTimes(LedSubsystem.kDefaultNotificationColor, 0.5, 2); }),
            new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, ElevatorAndArmConstants.ElevatorL3Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, ElevatorAndArmConstants.ArmRotateL3Posn))
    )); }
    public Command L2() { return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> { _LedSubsystem.setColorForDurationNTimes(LedSubsystem.kDefaultNotificationColor, 0.5, 3); }),
            new SetpointCommands.ElevatorToSetpointCommand((_ElevatorSubsystem), ElevatorAndArmConstants.ElevatorL2Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, ElevatorAndArmConstants.ArmRotateL2Posn))
    ));}
    public Command L1() { return new SequentialCommandGroup(new ParallelCommandGroup(
            new InstantCommand(() -> { _LedSubsystem.setColorForDurationNTimes(LedSubsystem.kDefaultNotificationColor, 0.5, 4); }),
            new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, ElevatorAndArmConstants.ElevatorL1Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, ElevatorAndArmConstants.ArmRotateL1Posn))
    )); }
    public Command ReturnHome() { return new SequentialCommandGroup(
            new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 0)
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, 0.0))
            .alongWith(new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, 0.0))
    ); }
    public Command CoralPickup() { return new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, ElevatorAndArmConstants.WristMax))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, ElevatorAndArmConstants.ArmRotateCoralPosn))
    ); }
}
