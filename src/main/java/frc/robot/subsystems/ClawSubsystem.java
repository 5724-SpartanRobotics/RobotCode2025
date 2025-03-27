package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;

public class ClawSubsystem extends SubsystemBase {
    private final SparkMax _Motor;
    private final Optional<LedSubsystem> _LedSubsystem;
    
    private double _LastTime = Timer.getFPGATimestamp();
    private boolean _ClawRunForDuration = false;
    private double _ClawRunDuration = 0.0;

    private ClawSubsystem(LedSubsystem ledSubsystem, boolean real) {
        _Motor = new SparkMax(Constants.CanIds.Claw, MotorType.kBrushless);
        _Motor.configure(new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit((int) Constants.Claw.CurrentLimit.in(Units.Amps))
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );

        _LedSubsystem = ledSubsystem != null ? Optional.of(ledSubsystem) : Optional.empty();
    }

    public ClawSubsystem() {
        this(null, true);
    }

    public ClawSubsystem(LedSubsystem ledSubsystem) {
        this(ledSubsystem, true);
    }

    @Override
    public void periodic() {
        if (DebugLevel.isOrAll(DebugLevel.Claw)) {
            SmartDashboard.putNumber("Claw/Amps", _Motor.getOutputCurrent());
        }

        if (isHighCurrentOutput()) _LedSubsystem.ifPresent(l -> l.setColor(LedSubsystem.kDefaultActiveColor));
        else _LedSubsystem.ifPresent(l -> l.reset());

        if (_ClawRunForDuration && Timer.getFPGATimestamp() >= _LastTime + _ClawRunDuration) {
            run(Constants.Claw.Speeds.Stopped);
            _ClawRunForDuration = false;
        }
    }

    public void run(double speed) {
        if (DebugLevel.isOrAll(DebugLevel.Claw)) {
            SmartDashboard.putNumber("Claw/Reference", speed);
        }

        _Motor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public void run(double speed, Time duration) {
        _ClawRunForDuration = true;
        _ClawRunDuration = duration.in(Units.Seconds);
        _LastTime = Timer.getFPGATimestamp();
        run(speed);
    }

    public boolean isHighCurrentOutput() {
        return _Motor.getOutputCurrent() >= Constants.Claw.HighCurrent.in(Units.Amps);
    }
}
