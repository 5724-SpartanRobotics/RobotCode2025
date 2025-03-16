package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class PresetCommands {
    public final SequentialCommandGroup L4;
    public final SequentialCommandGroup L3;
    public final SequentialCommandGroup L2;
    public final SequentialCommandGroup L1;
    public final SequentialCommandGroup ReturnHome;
    public final SequentialCommandGroup CoralPickup;

    public PresetCommands(
        ElevatorSubsystem elevatorSubsystem,
        WristSubsystem wristSubsystem,
        ArmSubsystem armSubsystem
    ) {
        L4 = new SequentialCommandGroup(
            new WaitCommand(0)
            .alongWith(new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn))
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateL4Posn))
        );
        L3 = new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL3Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateL3Posn))
        );
        L2 = new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand((elevatorSubsystem), ElevatorAndArmConstants.ElevatorL2Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateL2Posn))
        );
        L1 = new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL1Posn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateL1Posn))
        );
        ReturnHome = new SequentialCommandGroup(
            new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, 0)
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, 0.0))
            .alongWith(new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, 0.0))
        );
        CoralPickup = new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn)
            .alongWith(new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, ElevatorAndArmConstants.WristMax))
            .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateCoralPosn))
        );
    }
}
