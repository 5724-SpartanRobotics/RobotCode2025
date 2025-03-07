package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubSys;

public class WristRotateToSetpointCommand extends Command{
    private WristSubSys _WristSubSys;
    private double _Setpoint;

    public WristRotateToSetpointCommand(WristSubSys wrist, double setpoint){
        _WristSubSys = wrist;
        _Setpoint = setpoint;
        addRequirements(_WristSubSys);
    }
    @Override
    public void execute() {
        _WristSubSys.WristToPosition(_Setpoint);
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
        _WristSubSys.WristStop();
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(_WristSubSys.GetWristDegrees() - _Setpoint);
        return diff <= 10;
      }

}
