package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.lib.PidEnabledSubsystemInterface;
import frc.robot.lib.PidRamp;

public class ArmSubsystem extends SubsystemBase implements PidEnabledSubsystemInterface {
    private SparkMax _ArmRotateMtrCtrl1;
    private SparkMax _ArmRotateMtrCtrl2;

    private SparkClosedLoopController _ArmRotateMtr1PidController;
    private RelativeEncoder _ArmRotateMtr1Encoder;

    private PidRamp _ArmRotatePidRamp;
    private double _ArmRotateSetpoint;

    public ArmSubsystem()
    {
        _ArmRotateMtrCtrl1 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl1CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl2 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl2CanId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmRotatePidP, ElevatorAndArmConstants.ArmRotatePidI, ElevatorAndArmConstants.ArmRotatePidD)
        .velocityFF(ElevatorAndArmConstants.ArmRotatePidFF)
        .iMaxAccum(0.2);
        _ArmRotateMtrCtrl1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
        cfg//.inverted(false)
        .idleMode(IdleMode.kBrake)
        .follow(CanIdConstants.ArmRotateMtrCtrl1CanId, true);
        _ArmRotateMtrCtrl2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmRotatePidP, ElevatorAndArmConstants.ArmRotatePidI, ElevatorAndArmConstants.ArmRotatePidD)
        .velocityFF(ElevatorAndArmConstants.ArmRotatePidFF);
        _ArmRotateMtrCtrl1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ArmRotateMtr1PidController = _ArmRotateMtrCtrl1.getClosedLoopController();
        _ArmRotateMtr1Encoder = _ArmRotateMtrCtrl1.getEncoder();
        _ArmRotatePidRamp = new PidRamp(_ArmRotateMtr1PidController, ConvertArmRotateAngleToNeoRotations(ElevatorAndArmConstants.ArmRotateSetpointRampRate));

        _ArmRotateMtr1Encoder.setPosition(ConvertArmRotateAngleToNeoRotations(24.0));
    }

    @Override
    public void periodic() {
        super.periodic();
        double armRotatePosition = GetArmRotateAngleDegrees();

        _ArmRotatePidRamp.Periodic(ConvertArmRotateAngleToNeoRotations(armRotatePosition));

        if (DebugSetting.TraceLevel == DebugLevel.ArmRotate || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("ArmRotPos", armRotatePosition);
            SmartDashboard.putNumber("ArmRotRef", _ArmRotateSetpoint);
            SmartDashboard.putNumber("ArmRotRefRampped", _ArmRotatePidRamp.GetCurrentRampedSetpoint() * 360 / ElevatorAndArmConstants.ArmRotateGearRatio);
            SmartDashboard.putNumber("ArmRotCurrentM1", _ArmRotateMtrCtrl1.getOutputCurrent());
            SmartDashboard.putNumber("ArmRotCurrentM2", _ArmRotateMtrCtrl2.getOutputCurrent());
        }
    }

    public void resetReferences() {
        ArmRotateToPosition(0);
    }

    public double GetArmRotateAngleDegrees()
    {
        return _ArmRotateMtr1Encoder.getPosition() / ElevatorAndArmConstants.ArmRotateGearRatio * 360.0;
    }

    public double GetArmOutsideFrameInches()
    {
        double armRotation = 90 - GetArmRotateAngleDegrees();
        double armAngleToHorizontal = Math.abs(armRotation);
        return Math.cos(Math.toRadians(armAngleToHorizontal)) - ElevatorAndArmConstants.ArmPivotPointToFrameEdgeInches;
    }

    private double ConvertArmRotateAngleToNeoRotations(double angle)
    {
        return angle / 360 * ElevatorAndArmConstants.ArmRotateGearRatio;
    }
    public void ArmRotateStop()
    {
        _ArmRotatePidRamp.Stop();
    }

    public void ArmRotateToPosition(double degrees)
    {
        _ArmRotateSetpoint = degrees;
        if (_ArmRotateSetpoint > ElevatorAndArmConstants.ArmRotateMax)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMax;
        if (_ArmRotateSetpoint < ElevatorAndArmConstants.ArmRotateMin)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMin;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void ArmRotateToPositionMoreThanCurrent()
    {
        _ArmRotateSetpoint = GetArmRotateAngleDegrees() + 10.0;
        if (_ArmRotateSetpoint > ElevatorAndArmConstants.ArmRotateMax)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMax;
        if (_ArmRotateSetpoint < ElevatorAndArmConstants.ArmRotateMin)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMin;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void ArmRotateToPositionLessThanCurrent()
    {
        _ArmRotateSetpoint = GetArmRotateAngleDegrees() - 15.0;
        if (_ArmRotateSetpoint < ElevatorAndArmConstants.ArmRotateMin)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMin;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }
}
