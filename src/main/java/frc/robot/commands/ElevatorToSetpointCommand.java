package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToSetpointCommand extends Command{

    ElevatorSubsystem _ElevatorSubSys;
    double _Setpoint;
    public ElevatorToSetpointCommand(ElevatorSubsystem eleSubSys, double setpoint) {
        _ElevatorSubSys = eleSubSys;
        _Setpoint = setpoint;
        addRequirements(eleSubSys);
    }

    @Override
    public void execute() {
        _ElevatorSubSys.ElevatorToPosition(_Setpoint);
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            _ElevatorSubSys.ElevatorStop();
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(_ElevatorSubSys.GetElevatorHeightInches() - _Setpoint);
        return diff <= 1.0;
      }
}
