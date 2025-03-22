package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
        (new SetpointCommands.WristRotateToSetpointCommand(_WristSubsystem, ElevatorAndArmConstants.WristRotateHalf)).execute();
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
