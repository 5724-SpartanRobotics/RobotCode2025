package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Leave extends Command {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    Timer _StartTime;
    public Leave(DriveTrainSubsystem driveTrainSubsystem) {
        addRequirements(driveTrainSubsystem);
        _DriveTrainSubsystem = driveTrainSubsystem;
    }
    @Override
    public void execute() {
        if (_StartTime == null){
            _StartTime = new Timer();
            _StartTime.start();
        }
        _DriveTrainSubsystem.drive(new Translation2d(-0.3, 0).times(DriveConstants.maxRobotSpeedmps), 0);

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
