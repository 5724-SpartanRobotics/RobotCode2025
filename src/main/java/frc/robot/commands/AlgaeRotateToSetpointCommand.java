package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeRotateToSetpointCommand extends Command{

    private AlgaeSubsystem _AlgaeSubSystem;
    private double _Setpoint;

    public AlgaeRotateToSetpointCommand(AlgaeSubsystem algae, double setpoint)
    {
        _AlgaeSubSystem = algae;
        _Setpoint = setpoint;
        addRequirements(_AlgaeSubSystem);
    }

      @Override
      public void execute() {
          _AlgaeSubSystem.AlgaeToSetpoint(_Setpoint);
      }
    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            _AlgaeSubSystem.AlgaeStop();
    }
  
      @Override
      public boolean isFinished() {
          double diff = Math.abs(_AlgaeSubSystem.GetAlgaeArmAngleDegrees() - _Setpoint);
          return diff <= 3;
        }
  
}
