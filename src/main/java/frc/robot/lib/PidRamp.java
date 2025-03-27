package frc.robot.lib;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class PidRamp {
    private final SlewRateLimiter _Ramp;
    private final SparkClosedLoopController _Pid0;
    private final Optional<SparkClosedLoopController> _Pid1;

    private boolean _StopIsActive;
    private double _Setpoint;
    private double _RampedSetpoint;
    private double _CurrentPositionFeedback;

    public PidRamp(SparkClosedLoopController pidController, Measure<AngularVelocityUnit> rampRate) {
        _Pid0 = pidController;
        _Pid1 = Optional.empty();
        _Ramp = new SlewRateLimiter(rampRate.in(Units.RotationsPerSecond));
    }

    public PidRamp(SparkClosedLoopController pidController0, SparkClosedLoopController pidController1, Measure<AngularVelocityUnit> rampRate) {
        _Pid0 = pidController0;
        _Pid1 = Optional.of(pidController1);
        _Ramp = new SlewRateLimiter(rampRate.in(Units.RotationsPerSecond));
    }

    public void update() {
        if (_StopIsActive) {
            _Setpoint = _CurrentPositionFeedback;
            _RampedSetpoint = _Ramp.calculate(_CurrentPositionFeedback);
        } else {
            _RampedSetpoint = _Ramp.calculate(_Setpoint);
            _Pid0.setReference(_RampedSetpoint, ControlType.kPosition);
            if (_Pid1.isPresent()) _Pid1.get().setReference(_RampedSetpoint, ControlType.kPosition);
        }
    }

    public void update(double currentPositionFeedback) {
        setCurrentPositionFeedback(currentPositionFeedback);
        update();
    }

    /**
     * @param setpoint in motor rotation units
     */
    public void setReference(double setpoint) {
        _StopIsActive = false;
        _Setpoint = setpoint;
    }

    public void stop() {
        _StopIsActive = true;
    }

    public boolean getStopIsActive() {
        return _StopIsActive;
    }

    public double getCurrentRampedSetpoint() {
        return _RampedSetpoint;
    }

    public double getSetpoint() {
        return _Setpoint;
    }

    public double setCurrentPositionFeedback(double currentPositionFeedback) {
        return _CurrentPositionFeedback = currentPositionFeedback;
    }
}
