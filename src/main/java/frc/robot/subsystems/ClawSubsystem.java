package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;

public class ClawSubsystem extends SubsystemBase {
    private final LedSubsystem _LedSubsystem;
    private final SparkMax _ClawIntake;

    public ClawSubsystem(LedSubsystem ledSubsystem) {
        this._LedSubsystem = ledSubsystem;

        _ClawIntake = new SparkMax(CanIdConstants.ClawMtrCtrlCanId, MotorType.kBrushless);
         _ClawIntake.configure(new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit(50)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (ClawIsUsingLotsOfCurrent()) {
            _LedSubsystem.setColor(LedSubsystem.kDefaultActiveColor);
        } else {
            _LedSubsystem.reset();
        }
    }

    public boolean ClawIsUsingLotsOfCurrent() {
        return _ClawIntake.getOutputCurrent() > 15.0;
    }
    
    public void ClawRun(double speed)
    {
        if (DebugSetting.TraceLevel == DebugLevel.Claw || DebugSetting.TraceLevel == DebugLevel.All)
            SmartDashboard.putNumber("ClawSpeedRef", speed);
        _ClawIntake.set(speed);
    }
}
