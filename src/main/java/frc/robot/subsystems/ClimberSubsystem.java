package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.lib.PidRamp;

public class ClimberSubsystem extends SubsystemBase {
    private SparkFlex _ClimbMotor;
    private SparkClosedLoopController _ClimbPidController;
    private PidRamp _Ramp;

    public ClimberSubsystem() {
        _ClimbMotor = new SparkFlex(Constants.CanIdConstants.Climber, MotorType.kBrushless);
        _ClimbMotor.configure(new SparkFlexConfig()
            .inverted(false)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .apply(new ClosedLoopConfig().pidf(ClimberConstants.PidP, ClimberConstants.PidI, ClimberConstants.PidD, ClimberConstants.PidFF))
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );
        _ClimbPidController = _ClimbMotor.getClosedLoopController();
        _Ramp = new PidRamp(_ClimbPidController, null, ClimberConstants.RampRate);
    }

    public void SetToClimbPosition() {
        _Ramp.setReference(ConvertClimberDegreesToMotorRotations(ClimberConstants.ClimbAngle));
    }

    public void SetToHomePosition() {
        _Ramp.setReference(0);
    }

    public void Climb() {
        _ClimbMotor.set(0.5);
    }

    private double ConvertClimberDegreesToMotorRotations(double angle) {
        return angle / 360 * ClimberConstants.GearRatio;
    }
}
