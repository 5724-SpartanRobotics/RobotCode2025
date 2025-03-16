package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class SetpointCommands {
    public static class AlgaeRotateToSetpointCommand extends Command {
        private AlgaeSubsystem _AlgaeSubSystem;
        private double _Setpoint;

        public AlgaeRotateToSetpointCommand(AlgaeSubsystem algae, double setpoint) {
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
            if (interrupted) _AlgaeSubSystem.AlgaeStop();
        }
    
        @Override
        public boolean isFinished() {
            double diff = Math.abs(_AlgaeSubSystem.GetAlgaeArmAngleDegrees() - _Setpoint);
            return diff <= 5;
        }
    }

    public static class ArmRotateToSetpointCommand extends Command {
        private ArmSubsystem _ElevatorAndArmSubSys;
        private double _Setpoint;

        public ArmRotateToSetpointCommand(ArmSubsystem eleArmSs, double setpoint) {
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
            if (interrupted) _ElevatorAndArmSubSys.ArmRotateStop();
        }

        @Override
        public boolean isFinished() {
            double diff = Math.abs(_ElevatorAndArmSubSys.GetArmRotateAngleDegrees() - _Setpoint);
            return diff <= 10;
        }
    }

    public static class ElevatorToSetpointCommand extends Command {
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
            if (interrupted) _ElevatorSubSys.ElevatorStop();
        }

        @Override
        public boolean isFinished() {
            double diff = Math.abs(_ElevatorSubSys.GetElevatorHeightInches() - _Setpoint);
            return diff <= 1.0;
        }
    }

    public static class WristRotateToSetpointCommand extends Command {
        private WristSubsystem _WristSubSys;
        private double _Setpoint;

        public WristRotateToSetpointCommand(WristSubsystem wrist, double setpoint) {
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
            if (interrupted) _WristSubSys.WristStop();
        }

        @Override
        public boolean isFinished() {
            double diff = Math.abs(_WristSubSys.GetWristDegrees() - _Setpoint);
            return diff <= 10;
        }
    }
}
