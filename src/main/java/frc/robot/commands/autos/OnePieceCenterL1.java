package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ClawRunForDurationCommand;
import frc.robot.commands.PresetCommands;
import frc.robot.commands.SetpointCommands;
import frc.robot.commands.ClawRunForDurationCommand.ClawRunMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class OnePieceCenterL1 extends Command {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ElevatorSubsystem _ElevatorSubsystem;
    private ArmSubsystem _ArmSubsystem;
    private WristSubsystem _WristSubsystem;
    private ClawSubsystem _ClawSubsystem;
    private PresetCommands _PresetCommands;
    Timer _StartTime;

    public OnePieceCenterL1(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ClawSubsystem clawSubsystem, PresetCommands presetCommands) {
        addRequirements(driveTrainSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, clawSubsystem);
        _DriveTrainSubsystem = driveTrainSubsystem;
        _ElevatorSubsystem = elevatorSubsystem;
        _ArmSubsystem = armSubsystem;
        _WristSubsystem = wristSubsystem;
        _ClawSubsystem = clawSubsystem;
        _PresetCommands = presetCommands;
    }

    @Override
    public void execute() {
        if (_StartTime == null){
            _StartTime = new Timer();
            _StartTime.start();
        }
        Commands.sequence(
            Commands.parallel(
                new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, ElevatorAndArmConstants.WristRotateHalf).withDeadline(new WaitCommand(0.5)),
                (new ClawRunForDurationCommand(_ClawSubsystem, ClawRunMode.Intake, 0.5)).withDeadline(new WaitCommand(.5))
            ),
            Commands.parallel(
                Commands.sequence(
                new ParallelDeadlineGroup(new WaitCommand(4.330), new InstantCommand(() -> {
                    _DriveTrainSubsystem.drive(new Translation2d(0.09, 0).times(DriveConstants.maxRobotSpeedmps), 0);
                }, _DriveTrainSubsystem)),
                _DriveTrainSubsystem.brakeCmd()
                ),
                new SequentialCommandGroup(
                new WaitCommand(0)
                .alongWith(new SetpointCommands.ElevatorToSetpointCommand(_ElevatorSubsystem, ElevatorAndArmConstants.ElevatorL1Posn))
                .alongWith(new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, 80))
                .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(_ArmSubsystem, 65.0))
                ).withDeadline(new WaitCommand(2.5))
            ),
            (new ClawRunForDurationCommand(_ClawSubsystem, ClawRunMode.OuttakeDblSpeed, 1.5)).withDeadline(new WaitCommand(1.5)),
            new WaitCommand(0.5),
            Commands.sequence(
                new ParallelDeadlineGroup(new WaitCommand(1.0), new InstantCommand(() -> {
                    _DriveTrainSubsystem.drive(new Translation2d(-0.15, 0.15).times(DriveConstants.maxRobotSpeedmps), 0);
                }, _DriveTrainSubsystem)),
                _DriveTrainSubsystem.brakeCmd()
            ),
            new InstantCommand(() -> {_DriveTrainSubsystem.flipGyro();}, _DriveTrainSubsystem),
            _PresetCommands.ReturnHome()
        ).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        _DriveTrainSubsystem.drive(new Translation2d(0.0, 0).times(DriveConstants.maxRobotSpeedmps), 0);

        _StartTime.stop();
        _StartTime = null;
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    public boolean isFinished() {
        return _StartTime.hasElapsed(2.0);
    }
}
