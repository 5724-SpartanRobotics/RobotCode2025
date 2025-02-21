package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndArmSubSys;

public class ArmRotateToSetpointCommand extends Command{

    private ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    private double _Setpoint;
    public ArmRotateToSetpointCommand(ElevatorAndArmSubSys eleArmSs, double setpoint)
    {
        _ElevatorAndArmSubSys = eleArmSs;
        _Setpoint = setpoint;
        addRequirements(_ElevatorAndArmSubSys);
    }

    @Override
    public void execute() {
        _ElevatorAndArmSubSys.ArmRotateToPosition(_Setpoint);
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            _ElevatorAndArmSubSys.ArmRotateStop();
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(_ElevatorAndArmSubSys.GetArmRotateAngleDegrees() - _Setpoint);
        return diff <= 10;
      }

}
