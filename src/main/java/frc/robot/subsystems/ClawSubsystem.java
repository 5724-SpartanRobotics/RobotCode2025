package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;

public class ClawSubsystem extends SubsystemBase {
    private final LedSubsystem _LedSubsystem;
    private final SparkMax _ClawIntake;
    private final Timer _Timer = new Timer();
    private boolean _ClawRunForDuration = false;
    private double _ClawRunDuration = 0.0;

    public ClawSubsystem(LedSubsystem ledSubsystem) {
        this._LedSubsystem = ledSubsystem;

        _ClawIntake = new SparkMax(CanIdConstants.ClawMtrCtrlCanId, MotorType.kBrushless);
        _ClawIntake.configure(new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit(32)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _Timer.start();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (ClawIsUsingLotsOfCurrent()) {
            _LedSubsystem.setColor(LedSubsystem.kDefaultActiveColor);
        } else {
            _LedSubsystem.reset();
        }

        if (_ClawRunForDuration && _Timer.hasElapsed(_ClawRunDuration)) {
            ClawRun(ClawConstants.StoppedSpeed);
            _ClawRunForDuration = false;
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

    public void ClawRunForDuration(double seconds, double speed) {
        _Timer.stop(); _Timer.reset(); _Timer.start();
        _ClawRunForDuration = true;
        _ClawRunDuration = seconds;
        ClawRun(speed);
    }
}
