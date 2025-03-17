package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawRunForDurationCommand extends Command {
    private final Timer _time = new Timer();
    private final double _elapse;
    private final ClawSubsystem _ClawSubsystem;
    private final ClawRunMode _ClawRunMode;
    
    public ClawRunForDurationCommand(ClawSubsystem clawSubsystem, ClawRunMode intakeMode, double seconds) {
        _ClawSubsystem = clawSubsystem;
        _ClawRunMode = intakeMode;
        _elapse = seconds;
        _time.start();
    }

    private double ClawRunModeToDouble(ClawRunMode clawRunMode) {
        switch (clawRunMode) {
            case Intake: return ClawConstants.IntakeSpeed;
            case Outtake: return ClawConstants.ExpelSpeed;
            case Stopped: return ClawConstants.StoppedSpeed;
            default: return ClawConstants.StoppedSpeed;
        }
    }

    @Override
    public void execute() {
        _ClawSubsystem.ClawRun(ClawRunModeToDouble(_ClawRunMode));
    }

    @Override
    public boolean isFinished() {
        return _time.hasElapsed(_elapse);
    }

    @Override
    public void end(boolean interrupted) {
        _ClawSubsystem.ClawRun(ClawConstants.StoppedSpeed);
    }

    public static enum ClawRunMode {
        Intake,
        Outtake,
        Stopped
    }
}
