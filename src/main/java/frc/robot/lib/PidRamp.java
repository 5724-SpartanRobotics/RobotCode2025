package frc.robot.lib;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * A ramp of a setpoint. The ramped output feeds a SmartMax motor controller
 */
public class PidRamp {

    private SlewRateLimiter _Ramp;
    private SparkClosedLoopController _Pid;
    private boolean _StopIsActive;
    private double _Setpoint;
    private double _RampedSetpoint;
    /**
     * Creates a new PID ramp using the given units/second rate
     * @param pidController - The PID controller
     * @param rampRate - The ramp rate. The rate is in engineering units. For example, if your PID setpoint is in
     *   inches, this ramp rate would be in inches per second.
     */
    public PidRamp(SparkClosedLoopController pidController, double rampRate) {
        _Pid = pidController;
        _Ramp = new SlewRateLimiter(rampRate);
    }

    public void Stop() {
        _StopIsActive = true;
        _Pid.setReference(0, ControlType.kVelocity);
    }

    public void SetReference(double setpoint) {
        _StopIsActive = false;
        _Setpoint = setpoint;
    }

    /**
     * Call this method in your periodic method. The current setpoint will be ramped and sent
     * to the PID controller. If stop is active, the ramp input will be set to the current
     * position feedback.
     * @param currentPositionFeedback
     */
    public void Periodic(double currentPositionFeedback){
        if (_StopIsActive)
        {
            _Setpoint = currentPositionFeedback;
            _RampedSetpoint = _Ramp.calculate(currentPositionFeedback);
        }
        else
        {
            _RampedSetpoint = _Ramp.calculate(_Setpoint);
            _Pid.setReference(_RampedSetpoint, ControlType.kPosition);
        }
    }

    public boolean GetStopIsActive(){
        return _StopIsActive;
    }

    public double GetCurrentRampedSetpoint(){
        return _RampedSetpoint;
    }

    public double GetSetpoint(){
        return _Setpoint;
    }
}
