package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.SetpointCommands;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class LeaveWithWrist extends Command {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private WristSubsystem _WristSubsystem;
    Timer _StartTime;
    public LeaveWithWrist(DriveTrainSubsystem driveTrainSubsystem, WristSubsystem wristSubsystem) {
        addRequirements(driveTrainSubsystem, wristSubsystem);
        _DriveTrainSubsystem = driveTrainSubsystem;
        _WristSubsystem = wristSubsystem;
    }
    @Override
    public void execute() {
        if (_StartTime == null){
            _StartTime = new Timer();
            _StartTime.start();
        }
        Commands.parallel(
          new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, ElevatorAndArmConstants.WristRotateHalf),
          Commands.sequence(
            new ParallelDeadlineGroup(
              new WaitCommand(3),
              new RunCommand(() -> {_DriveTrainSubsystem.drive(new Translation2d(-0.3, 0).times(DriveConstants.maxRobotSpeedmps), 0);}, _DriveTrainSubsystem)
            ),
            _DriveTrainSubsystem.brakeCmd()
          )          
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
    return _StartTime.hasElapsed(3);
  }
}
