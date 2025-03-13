package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawSubsystem;

public class ClawRunForDurationCommand extends SequentialCommandGroup {
    public ClawRunForDurationCommand(ClawSubsystem clawSubsystem, ClawRunMode intakeMode, double seconds) {
        addCommands(
            Commands.deadline(new WaitCommand(seconds), new RunCommand(() -> {clawSubsystem.ClawRun(ClawRunModeToDouble(intakeMode));}, clawSubsystem)),
            new RunCommand(() -> {clawSubsystem.ClawRun(0.0);}, clawSubsystem)
        );
    }

    private double ClawRunModeToDouble(ClawRunMode clawRunMode) {
        switch (clawRunMode) {
            case Intake: return -0.3;
            case Outtake: return 0.3;
            case Stopped: return 0.0;
            default: return 0.0;
        }
    }

    public static enum ClawRunMode {
        Intake,
        Outtake,
        Stopped
    }
}
