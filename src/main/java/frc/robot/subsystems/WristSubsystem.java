package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;
import frc.robot.lib.PidRamp;
import frc.robot.lib.PidfEnabledSubsystemInterface;
import frc.robot.util.Conversions;

public class WristSubsystem extends SubsystemBase implements PidfEnabledSubsystemInterface {
    private final SparkFlex _Motor;
    private final SparkClosedLoopController _PidController;
    private final RelativeEncoder _Encoder;
    private final PidRamp _Ramp;

    private double _RotateSetpoint;

    public WristSubsystem() {
        _Motor = new SparkFlex(Constants.CanIds.Wrist, MotorType.kBrushless);
        _Motor.configure(new SparkFlexConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Wrist.PIDFf.getP(), Constants.Wrist.PIDFf.getI(), Constants.Wrist.PIDFf.getD(), Constants.Wrist.PIDFf.getFf())
                .iMaxAccum(Constants.Wrist.IntegralMaxAccumulation)
            ),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );

        _PidController = _Motor.getClosedLoopController();
        _Encoder = _Motor.getEncoder();
        _Ramp = PidRamp.fromAngular(_PidController, Constants.Wrist.RampRate.times(Constants.Wrist.GearRatio));
    }

    @Override
    public void periodic() {
        Angle position = getAngle();
        _Ramp.setCurrentPositionFeedback(calculateSetpoint(position.in(Units.Degrees)));

        if (DebugLevel.isOrAll(DebugLevel.Wrist)) {
            SmartDashboard.putNumber("Wrist/Position", position.in(Units.Degrees));
            SmartDashboard.putNumber("Wrist/Reference", _RotateSetpoint);
            SmartDashboard.putNumber("Wrist/RampReference", _Ramp.getCurrentRampedSetpoint());
        }
    }

    @Override
    public void resetReferences() {
        rotateTo(RotatePosition.In);
    }

    public void rotateTo(RotatePosition rotatePosition) {
        _RotateSetpoint = Conversions.clampAngle(rotatePosition.getPosition(), Constants.Wrist.RotateMin, Constants.Wrist.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void rotateTo(Angle angle) {
        _RotateSetpoint = Conversions.clampAngle(angle, Constants.Wrist.RotateMin, Constants.Wrist.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void stop() {
        _Ramp.stop();
    }

    public void incrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).plus(Constants.Wrist.Increment),
            Constants.Wrist.RotateMin, Constants.Wrist.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void decrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).minus(Constants.Wrist.Decrement),
            Constants.Wrist.RotateMin, Constants.Wrist.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public Angle getAngle() {
        return Units.Degrees.of(calculateSetpoint(_Encoder.getPosition()));
    }

    /**
     * @param value degrees
     * @return setpoint
     */
    @Override
    public double calculateSetpoint(double value) {
        return value / 360.0 * Constants.Wrist.GearRatio;
    }

    public static enum RotatePosition {
        In(Constants.Wrist.Positions.In),
        Out(Constants.Wrist.Positions.Out);

        private Angle pos;
        RotatePosition(Angle rotatePosition) {
            this.pos = rotatePosition;
        }
        public Angle getPosition() { return pos; }
    }


    public Command toSetpoint(RotatePosition setpoint) {
        WristSubsystem subsystem = this;
        return new Command() {
            @Override
            public void execute() {
                subsystem.toSetpoint(setpoint);
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) subsystem.stop();
            }

            @Override
            public boolean isFinished() {
                return Math.abs(subsystem.getAngle().minus(setpoint.getPosition()).in(Units.Degrees)) <= Constants.Wrist.RotateAccuracyThreshold.in(Units.Degrees);
            }
        };
    }
}
