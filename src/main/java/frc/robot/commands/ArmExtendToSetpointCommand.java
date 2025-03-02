package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubSys;

public class ArmExtendToSetpointCommand extends Command{

    private ArmSubSys _ElevatorAndArmSubSystem;
    private double _Setpoint;

    public ArmExtendToSetpointCommand(ArmSubSys eleArmSs, double setpoint)
    {
        _ElevatorAndArmSubSystem = eleArmSs;
        _Setpoint = setpoint;
        addRequirements(_ElevatorAndArmSubSystem);
    }
    @Override
    public void execute() {
        _ElevatorAndArmSubSystem.ArmExtendToPosition(_Setpoint);
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            _ElevatorAndArmSubSystem.ArmExtendStop();
    }

    @Override
    public boolean isFinished() {
        double diff = Math.abs(_ElevatorAndArmSubSystem.GetArmExtendPosnInches() - _Setpoint);
        return diff <= 1.0;
      }
}
