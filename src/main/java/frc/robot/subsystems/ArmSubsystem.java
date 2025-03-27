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

public class ArmSubsystem extends SubsystemBase implements PidfEnabledSubsystemInterface {
    private final SparkMax _Motor0;
    private final SparkMax _Motor1;
    private final SparkClosedLoopController _PidController;
    private final RelativeEncoder _Encoder;
    private final PidRamp _Ramp;

    private double _RotateSetpoint;

    public ArmSubsystem() {
        _Motor0 = new SparkMax(Constants.CanIds.ArmRotateMotor1, MotorType.kBrushless);
        _Motor0.configure(new SparkMaxConfig()
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(Constants.Arm.PIDFf.getP(), Constants.Arm.PIDFf.getI(), Constants.Arm.PIDFf.getD(), Constants.Arm.PIDFf.getFf())
                .iMaxAccum(Constants.Arm.IntegralMaxAccumulation)
            ),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );

        _Motor1 = new SparkMax(Constants.CanIds.ArmRotateMotor2, MotorType.kBrushless);
        _Motor1.configure(new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .follow(Constants.CanIds.ArmRotateMotor1),
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters
        );

        _PidController = _Motor0.getClosedLoopController();
        _Encoder = _Motor0.getEncoder();
        _Ramp = PidRamp.fromAngular(_PidController, Constants.Arm.RampRate.times(Constants.Arm.GearRatio).times(Constants.Motor.NeoV1.CountsPerRevolution));

        _Encoder.setPosition(Conversions.angle2rotations(Units.Degrees.of(24.0), Constants.Arm.GearRatio));
    }

    @Override
    public void periodic() {
        Angle armRotatePosition = getAngle();
        _Ramp.setCurrentPositionFeedback(Conversions.angle2rotations(armRotatePosition, Constants.Arm.GearRatio));

        if (DebugLevel.isOrAll(DebugLevel.ArmRotate)) {
            SmartDashboard.putNumber("Arm/Position", armRotatePosition.in(Units.Degrees));
            SmartDashboard.putNumber("Arm/Reference", _RotateSetpoint);
            SmartDashboard.putNumber("Arm/RampReference", Units.Rotations.of(_Ramp.getCurrentRampedSetpoint()).div(Constants.Arm.GearRatio).in(Units.Degrees));
            SmartDashboard.putNumber("Arm/Amps_M1", _Motor0.getOutputCurrent());
            SmartDashboard.putNumber("Arm/Amps_M2", _Motor1.getOutputCurrent());
        }
    }

    @Override
    public void resetReferences() {
        this.rotateTo(RotatePosition.Home);
    }

    public void rotateTo(RotatePosition rotatePosition) {
        _RotateSetpoint = Conversions.clampAngle(rotatePosition.getPosition(), Constants.Arm.RotateMin, Constants.Arm.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void rotateTo(Angle angle) {
        _RotateSetpoint = Conversions.clampAngle(angle, Constants.Arm.RotateMin, Constants.Arm.RotateMax, Units.Degrees);
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void stop() {
        _Ramp.stop();
    }

    public void incrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).plus(Constants.Arm.Increment),
            Constants.Arm.RotateMin, Constants.Arm.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public void decrementPosition() {
        _RotateSetpoint = Conversions.clampAngle(
            Units.Degrees.of(_RotateSetpoint).minus(Constants.Arm.Decrement),
            Constants.Arm.RotateMin, Constants.Arm.RotateMax, Units.Degrees
        );
        _Ramp.setReference(calculateSetpoint(_RotateSetpoint));
    }

    public Angle getAngle() {
        return Units.Degrees.of(_Encoder.getPosition() / Constants.Arm.GearRatio * 360.0);
    }

    /**
     * @param value the rotate setpoint (in degrees)
     * @return setpoint
     */
    @Override
    public double calculateSetpoint(double value) {
        return value / 360.0 * Constants.Arm.GearRatio;
    }

    public static enum RotatePosition {
        Home(Constants.Arm.Positions.Home),
        L1(Constants.Arm.Positions.L1),
        L2(Constants.Arm.Positions.L2),
        L3(Constants.Arm.Positions.L3),
        L4(Constants.Arm.Positions.L4),
        Intake(Constants.Arm.Positions.Intake);

        private Angle pos;
        RotatePosition(Angle rotatePosition) {
            this.pos = rotatePosition;
        }
        public Angle getPosition() { return pos; }
    }


    public Command toSetpoint(RotatePosition setpoint) {
        ArmSubsystem subsystem = this;
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
                return Math.abs(subsystem.getAngle().minus(setpoint.getPosition()).in(Units.Degrees)) <= Constants.Arm.RotateAccuracyThreshold.in(Units.Degrees);
            }
        };
    }
}
