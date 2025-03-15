package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawRunForDurationCommand extends SequentialCommandGroup {
    public ClawRunForDurationCommand(ClawSubsystem clawSubsystem, ClawRunMode intakeMode, double seconds) {
        addCommands(
            Commands.deadline(new WaitCommand(seconds), new RunCommand(() -> {clawSubsystem.ClawRun(ClawRunModeToDouble(intakeMode));}, clawSubsystem)),
            new RunCommand(() -> {clawSubsystem.ClawRun(ClawConstants.StoppedSpeed);}, clawSubsystem)
        );
    }

    private double ClawRunModeToDouble(ClawRunMode clawRunMode) {
        switch (clawRunMode) {
            case Intake: return ClawConstants.IntakeSpeed;
            case Outtake: return ClawConstants.ExpelSpeed;
            case Stopped: return ClawConstants.StoppedSpeed;
            default: return ClawConstants.StoppedSpeed;
        }
    }

    public static enum ClawRunMode {
        Intake,
        Outtake,
        Stopped
    }
}
