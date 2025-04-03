package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class OnePieceCenter extends Command {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ElevatorSubsystem _ElevatorSubsystem;
    private ArmSubsystem _ArmSubsystem;
    private WristSubsystem _WristSubsystem;
    private ClawSubsystem _ClawSubsystem;
    private Optional<Timer> _StartTime = Optional.empty();

    public OnePieceCenter(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ClawSubsystem clawSubsystem) {
        addRequirements(driveTrainSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, clawSubsystem);
        _DriveTrainSubsystem = driveTrainSubsystem;
        _ElevatorSubsystem = elevatorSubsystem;
        _ArmSubsystem = armSubsystem;
        _WristSubsystem = wristSubsystem;
        _ClawSubsystem = clawSubsystem;
    }

    // TODO: Check whether we should be using field relative driving or not.
    @Override
    public void execute() {
        if (_StartTime == null || _StartTime.isEmpty()){
            _StartTime = Optional.of(new Timer());
            _StartTime.get().start();
        }
        Commands.sequence(
            Commands.parallel(
                _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.Half),
                _ClawSubsystem.runForDurationCommand(Units.Seconds.of(0.5), ClawSubsystem.IntakeMode.Intake)
            ),
            Commands.parallel(
                Commands.sequence(
                    _DriveTrainSubsystem.driveCmd(new Translation2d(0.09, 0), 0, true).withDeadline(new WaitCommand(4.330)),
                    _DriveTrainSubsystem.brakeCmd()
                ),
                Commands.parallel(
                    _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.L1),
                    _WristSubsystem.toSetpoint(Units.Degrees.of(80.0)),
                    _ArmSubsystem.toSetpoint(Units.Degrees.of(65.0))
                ).withDeadline(new WaitCommand(2.5))
            ),
            _ClawSubsystem.runForDurationCommand(Units.Seconds.of(1.5), ClawSubsystem.IntakeMode.ExpelDoubleSpeed),
            new WaitCommand(0.5),
            Commands.sequence(
                _DriveTrainSubsystem.driveCmd(new Translation2d(-0.15, 0.15), 0, true).withDeadline(new WaitCommand(1.0)),
                _DriveTrainSubsystem.brakeCmd()
            ),
            _DriveTrainSubsystem.flipGyroCmd()
        ).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        _DriveTrainSubsystem.drive(new Translation2d(0.0, 0), 0, true);

        _StartTime.get().stop();
        _StartTime = Optional.empty();
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    public boolean isFinished() {
        return _StartTime.get().hasElapsed(2.0);
    }
}
