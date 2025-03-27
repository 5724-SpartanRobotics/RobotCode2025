package frc.robot.subsystems;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;
import frc.robot.lib.PidRamp;
import frc.robot.lib.PidfEnabledSubsystemInterface;
import frc.robot.util.Conversions;

public class AlgaeSubsystem extends SubsystemBase implements PidfEnabledSubsystemInterface {
    private final SparkMax _Motor;
    private final SparkClosedLoopController _PidController;
    private final RelativeEncoder _Encoder;
    private final PidRamp _Ramp;

    private double _RotateSetpoint;

    public AlgaeSubsystem() {
        _Motor = new SparkMax(Constants.CanIds.Algae, MotorType.kBrushless);
        _Motor.configure(new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Algae.PIDFf.getP(), Constants.Algae.PIDFf.getI(), Constants.Algae.PIDFf.getD(), Constants.Algae.PIDFf.getFf())
                .iMaxAccum(Constants.Algae.IntegralMaxAccumulation)
            ),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );
        _PidController = _Motor.getClosedLoopController();
        _Encoder = _Motor.getEncoder();
        _Ramp = PidRamp.fromAngular(_PidController, Constants.Algae.RampRate.times(Constants.Algae.GearRatio).times(Constants.Motor.NeoV1.CountsPerRevolution));
    }

    @Override
    public void periodic() {
        double algaeRotatePosition = getAngle().in(Units.Degrees);
        double motorPosition = algaeRotatePosition * Constants.Motor.NeoV1.CountsPerRevolution * Constants.Algae.GearRatio;
        _Ramp.setCurrentPositionFeedback(motorPosition);

        if (DebugLevel.isOrAll(DebugLevel.Algae)) {
            SmartDashboard.putNumber("Algae/Position", algaeRotatePosition);
            SmartDashboard.putNumber("Algae/Reference", _RotateSetpoint);
            SmartDashboard.putNumber("Algae/RampReference", _Ramp.getCurrentRampedSetpoint());
            SmartDashboard.putNumber("Algae/CurrentDraw", _Motor.getOutputCurrent());
            SmartDashboard.putBoolean("Algae/RampStopActive", _Ramp.getStopIsActive());
        }
    }

    @Override
    public void resetReferences() {
        this.rotateTo(RotatePosition.In);
    }

    public void rotateTo(RotatePosition rotatePosition) {
        _RotateSetpoint = Conversions.clampAngle(rotatePosition.getPosition(), Constants.Algae.RotateMin, Constants.Algae.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void rotateTo(Angle angle) {
        _RotateSetpoint = Conversions.clampAngle(angle, Constants.Algae.RotateMin, Constants.Algae.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void stop() {
        _Ramp.stop();
    }

    public void incrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).plus(Constants.Algae.Increment),
            Constants.Algae.RotateMin, Constants.Algae.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void decrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).minus(Constants.Algae.Decrement),
            Constants.Algae.RotateMin, Constants.Algae.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    /**
     * Automatically clamps the speed between -1 and 1
     * @param speed
     */
    public void rotateAtSpeed(double speed) {
        _Ramp.stop();
        _Motor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public Angle getAngle() {
        return Units.Degrees.of(_Encoder.getPosition() / Constants.Algae.GearRatio * 360.0);
    }

    /**
     * @param value the rotate setpoint (in degrees)
     * @return setpoint
     */
    @Override
    public double calculateSetpoint(double value) {
        return value * Constants.Motor.NeoV1.CountsPerRevolution * Constants.Algae.GearRatio / 360.0;
    }

    public static enum RotatePosition {
        In(Constants.Algae.Positions.Retract),
        Out(Constants.Algae.Positions.Extend),
        Hold(Constants.Algae.Positions.Hold);

        private Angle pos;
        RotatePosition(Angle rotatePosition) {
            this.pos = rotatePosition;
        }
        public Angle getPosition() { return pos; }
    }
}
