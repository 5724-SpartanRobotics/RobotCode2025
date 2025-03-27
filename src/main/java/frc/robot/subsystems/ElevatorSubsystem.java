package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;
import frc.robot.lib.PidRamp;
import frc.robot.lib.PidfEnabledSubsystemInterface;
import frc.robot.util.Conversions;

public class ElevatorSubsystem extends SubsystemBase implements PidfEnabledSubsystemInterface {
    private final SparkMax _Motor0;
    private final SparkMax _Motor1;
    private final SparkClosedLoopController _PidController0;
    private final RelativeEncoder _Encoder0;
    private final SparkClosedLoopController _PidController1;
    private final RelativeEncoder _Encoder1;
    private final PidRamp _Ramp;

    private double _Setpoint;
    private Instant _LastStall = Instant.MIN;
    private double _PositionLastStall;

    public ElevatorSubsystem() {
        _Motor0 = new SparkMax(Constants.CanIds.ElevatorMotor1, MotorType.kBrushless);
        _Motor0.configure(new SparkMaxConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Elevator.PIDFf.getP(), Constants.Elevator.PIDFf.getI(), Constants.Elevator.PIDFf.getI(), Constants.Elevator.PIDFf.getFf())
            ),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );

        _Motor1 = new SparkMax(Constants.CanIds.ElevatorMotor2, MotorType.kBrushless);
        _Motor1.configure(new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Elevator.PIDFf.getP(), Constants.Elevator.PIDFf.getI(), Constants.Elevator.PIDFf.getI(), Constants.Elevator.PIDFf.getFf())
            ),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );

        _PidController0 = _Motor0.getClosedLoopController();
        _PidController1 = _Motor1.getClosedLoopController();
        _Encoder0 = _Motor0.getEncoder();
        _Encoder1 = _Motor1.getEncoder();

        _Ramp = PidRamp.fromLinear(
            _PidController0, _PidController1,
            Constants.Elevator.RampRate
                .times(Units.Inches.of(1.0).div(Constants.Elevator.ChainSprocketDiameter.times(Math.PI)))
                // ^ We cannot use .div because that returns a generic type. We have to do 1.div because there is no reciprocal function.
                .times(Constants.Elevator.GearRatio)
        );
    }

    @Override
    public void periodic() {
        Distance height = getHeight();
        boolean skipped = likelySufferedChainJump();
        _Ramp.setCurrentPositionFeedback(calculateSetpoint(height.in(Units.Inches)));

        if (DebugLevel.isOrAll(DebugLevel.Elevator)) {
            SmartDashboard.putNumber("Elevator/Position_M0", height.in(Units.Inches));
            SmartDashboard.putNumber("Elevator/Position_M1", getHeight(true).in(Units.Inches));
            SmartDashboard.putNumber("Elevator/Reference", _Setpoint);
        }

        SmartDashboard.putBoolean("Elevator/Stalled", skipped);
        if (skipped) _Encoder0.setPosition(Position.Home.getDistance().in(Units.Inches));
    }

    @Override
    public void resetReferences() {
        to(Position.Home);
    }

    public void to(Position position) {
        _Setpoint = Conversions.clampDistance(position.getDistance(), Constants.Elevator.ExtendMin, Constants.Elevator.ExtendMax, Units.Inches);
        _Ramp.setReference(calculateSetpoint(_Setpoint));
    }

    public void to(Distance height) {
        _Setpoint = Conversions.clampDistance(height, Constants.Elevator.ExtendMin, Constants.Elevator.ExtendMax, Units.Inches);
        _Ramp.setReference(calculateSetpoint(_Setpoint));
    }

    public void stop() {
        _Ramp.stop();
    }

    public Distance getHeight() {
        return Units.Inches.of(calculateSetpoint(_Encoder0.getPosition()));
    }

    public Distance getHeight(Boolean motor2) {
        return (motor2 == null || !motor2.booleanValue()) ?
            getHeight() :
            Units.Inches.of(calculateSetpoint(_Encoder1.getPosition()));
    }

    public void incrementPosition() {
        _Setpoint = Conversions.clampDistance(
            Units.Inches.of(_Setpoint).plus(Constants.Elevator.Increment),
            Constants.Elevator.ExtendMin, Constants.Elevator.ExtendMax, Units.Inches
        );
        _Ramp.setReference(calculateSetpoint(_Setpoint));
    }

    public void decrementPosition() {
        _Setpoint = Conversions.clampDistance(
            Units.Inches.of(_Setpoint).minus(Constants.Elevator.Decrement),
            Constants.Elevator.ExtendMin, Constants.Elevator.ExtendMax, Units.Inches
        );
        _Ramp.setReference(calculateSetpoint(_Setpoint));
    }

    public boolean likelySufferedChainJump() {
        double position = getHeight().in(Units.Inches);
        boolean stallLikely = _Ramp.getCurrentRampedSetpoint() == 0.0 &&
            Math.abs(_Motor0.getOutputCurrent()) >= Constants.Elevator.StallCurrent.in(Units.Amps)  &&
            Math.abs(position - _PositionLastStall) <= 0.1 && // this is an indication of the elevator not moving
            position > Units.Inches.of(0.75).in(Units.Inches);
        _PositionLastStall = position;

        if (_LastStall == Instant.MIN && stallLikely) _LastStall = Instant.now();
        else if (!stallLikely) _LastStall = Instant.MIN;

        return _LastStall != Instant.MIN && Duration.between(_LastStall, Instant.now()).compareTo(Duration.ofSeconds(3L)) > 0;
    }

    /**
     * @param value height in inches
     * @return setpoint
     */
    @Override
    public double calculateSetpoint(double value) {
        return value / (Math.PI * Constants.Elevator.ChainSprocketDiameter.in(Units.Inches)) * Constants.Elevator.GearRatio;
    }

    public static enum Position {
        Home(Units.Inches.of(0.0));

        private Distance dist;
        Position(Distance distance) {
            this.dist = distance;
        }
        public Distance getDistance() { return dist; }
    }
}
