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
    }

    private double ClawRunModeToDouble(ClawRunMode clawRunMode) {
        return switch (clawRunMode) {
            case Intake -> ClawConstants.IntakeSpeed;
            case Outtake -> ClawConstants.ExpelSpeed;
            case OuttakeDlbSpeed -> ClawConstants.ExpelSpeed * 2;
            case Stopped -> ClawConstants.StoppedSpeed;
            default -> ClawConstants.StoppedSpeed;
        };
    }

    @Override
    public void execute() {
        _time.reset();
        _time.start();
        _ClawSubsystem.ClawRun(ClawRunModeToDouble(_ClawRunMode));
    }

    @Override
    public boolean isFinished() {
        return _time.hasElapsed(_elapse);
    }

    @Override
    public void end(boolean interrupted) {
        _time.stop();
        System.out.println("!~~ Claw finished running command");
        _ClawSubsystem.ClawRun(ClawRunModeToDouble(ClawRunMode.Stopped));
    }

    public static enum ClawRunMode {
        Intake,
        Outtake,
        OuttakeDlbSpeed,
        Stopped
    }
}
