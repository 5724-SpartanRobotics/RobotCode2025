package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAndArmSubSys;

public class ElevatorToSetpointCommand extends Command{

    ElevatorAndArmSubSys _ElevatorAndArmSubSys;
    double _Setpoint;
    public ElevatorToSetpointCommand(ElevatorAndArmSubSys eleArmSubSys, double setpoint) {
        _ElevatorAndArmSubSys = eleArmSubSys;
        _Setpoint = setpoint;
        addRequirements(eleArmSubSys);
    }

    @Override
    public void execute() {
        _ElevatorAndArmSubSys.ElevatorToPosition(_Setpoint);
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            _ElevatorAndArmSubSys.ElevatorStop();
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(_ElevatorAndArmSubSys.GetElevatorHeightInches() - _Setpoint);
        return diff <= 1.0;
      }
}
