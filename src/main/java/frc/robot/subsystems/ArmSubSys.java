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
import frc.robot.lib.PidRamp;

public class ArmSubSys extends SubsystemBase {
    private SparkMax _ArmRotateMtrCtrl1;
    private SparkMax _ArmRotateMtrCtrl2;
    private SparkMax _ArmExtendMtrCtrl;

    private SparkClosedLoopController _ArmRotateMtr1PidController;
    private RelativeEncoder _ArmRotateMtr1Encoder;
    private SparkClosedLoopController _ArmExtendMtrPidController;
    private RelativeEncoder _ArmExtenMtrEncoder;

    private PidRamp _ArmRotatePidRamp;
    private PidRamp _ArmExtendPidRamp;

    private double _ArmRotateSetpoint;
    private double _ArmExtendSetpoint;

    public ArmSubSys()
    {
        _ArmRotateMtrCtrl1 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl1CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl2 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl2CanId, MotorType.kBrushless);
        _ArmExtendMtrCtrl = new SparkMax(CanIdConstants.ArmExtendMtrCtrlCanId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmRotatePidP, ElevatorAndArmConstants.ArmRotatePidI, ElevatorAndArmConstants.ArmRotatePidD)
        .velocityFF(ElevatorAndArmConstants.ArmRotatePidFF);
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


        cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmExtendPidP, ElevatorAndArmConstants.ArmExtendPidI, ElevatorAndArmConstants.ArmExtendPidD)
        .velocityFF(ElevatorAndArmConstants.ArmExtendPidFF);
        _ArmExtendMtrCtrl.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ArmRotateMtr1PidController = _ArmRotateMtrCtrl1.getClosedLoopController();
        _ArmRotateMtr1Encoder = _ArmRotateMtrCtrl1.getEncoder();
        _ArmExtendMtrPidController = _ArmExtendMtrCtrl.getClosedLoopController();
        _ArmExtenMtrEncoder = _ArmExtendMtrCtrl.getEncoder();
        _ArmRotatePidRamp = new PidRamp(_ArmRotateMtr1PidController, null, ConvertArmRotateAngleToNeoRotations(ElevatorAndArmConstants.ArmRotateSetpointRampRate));
        _ArmExtendPidRamp = new PidRamp(_ArmExtendMtrPidController, null, ConvertArmExtendInchesToRotations(ElevatorAndArmConstants.ArmExtendSetpointRampRate));
    }

    @Override
    public void periodic(){
        super.periodic();
        double armExtendPosition = GetArmExtendPosnInches();
        double armRotatePosition = GetArmRotateAngleDegrees();
        double outsideFrame = GetArmOutsideFrameInches();

        //Keep the arm within the frame paremeter
        if (outsideFrame > 17){
            _ArmRotatePidRamp.setReference(ConvertArmRotateAngleToNeoRotations(armRotatePosition));
            ArmExtendToPosition(armExtendPosition - 1);
        }

        _ArmRotatePidRamp.Periodic(ConvertArmRotateAngleToNeoRotations(armRotatePosition));
        _ArmExtendPidRamp.Periodic(ConvertArmExtendInchesToRotations(armExtendPosition));

        if (DebugSetting.TraceLevel == DebugLevel.ArmExtend || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("ArmExtPos", armExtendPosition);
            SmartDashboard.putNumber("ArmExtRef", _ArmExtendSetpoint);
            SmartDashboard.putNumber("ArmExtendRameOutRef", _ArmExtendPidRamp.GetCurrentRampedSetpoint());
        }
        if (DebugSetting.TraceLevel == DebugLevel.ArmRotate || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("ArmRotPos", armRotatePosition);
            SmartDashboard.putNumber("ArmRotRef", _ArmRotateSetpoint);
            SmartDashboard.putNumber("ArmRotCurrentM1", _ArmRotateMtrCtrl1.getOutputCurrent());
            SmartDashboard.putNumber("ArmRotCurrentM2", _ArmRotateMtrCtrl2.getOutputCurrent());
            SmartDashboard.putNumber("ArmOutsideFrame", outsideFrame);
        }
    }

    public double GetArmRotateAngleDegrees()
    {
        return _ArmRotateMtr1Encoder.getPosition() / ElevatorAndArmConstants.ArmRotateGearRatio * 360.0;
    }

    public double GetArmExtendPosnInches()
    {
        return _ArmExtenMtrEncoder.getPosition() / ElevatorAndArmConstants.ArmExtendGearRatio * Math.PI * ElevatorAndArmConstants.ArmExtendSpoolDiameter;
    }

    public double GetArmOutsideFrameInches()
    {
        double armCurrentLength = ElevatorAndArmConstants.ArmLengthRetractedInches + GetArmExtendPosnInches();
        double armRotation = 90 - GetArmRotateAngleDegrees();
        double armAngleToHorizontal = Math.abs(armRotation);
        return (Math.cos(Math.toRadians(armAngleToHorizontal))*armCurrentLength) - ElevatorAndArmConstants.ArmPivotPointToFrameEdgeInches;
    }

    private double ConvertArmRotateAngleToNeoRotations(double angle)
    {
        return angle / 360 * ElevatorAndArmConstants.ArmRotateGearRatio;
    }
    public void IncrementArmRotate()
    {
        _ArmRotateSetpoint += 5;
        if (_ArmRotateSetpoint > ElevatorAndArmConstants.ArmRotateMax)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMax;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }


    public void DecrementArmRotate()
    {
        _ArmRotateSetpoint -= 5;
        if (_ArmRotateSetpoint < 0)
            _ArmRotateSetpoint = 0;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
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
        if (_ArmRotateSetpoint < 0)
            _ArmRotateSetpoint = 0;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void ArmRotateToPositionMoreThanCurrent()
    {
        _ArmRotateSetpoint = GetArmRotateAngleDegrees() + 5.0;
        if (_ArmRotateSetpoint > ElevatorAndArmConstants.ArmRotateMax)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMax;
        if (_ArmRotateSetpoint < 0)
            _ArmRotateSetpoint = 0;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void ArmRotateToPositionLessThanCurrent()
    {
        _ArmRotateSetpoint = GetArmRotateAngleDegrees() - 5.0;
        if (_ArmRotateSetpoint > ElevatorAndArmConstants.ArmRotateMax)
            _ArmRotateSetpoint = ElevatorAndArmConstants.ArmRotateMax;
        if (_ArmRotateSetpoint < 0)
            _ArmRotateSetpoint = 0;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void IncrementArmExtend()
    {
        _ArmExtendSetpoint += 1;
        if (_ArmExtendSetpoint > ElevatorAndArmConstants.ArmExtendMax)
            _ArmExtendSetpoint = ElevatorAndArmConstants.ArmExtendMax;
        //convert from inches to motor rotations
        _ArmExtendPidRamp.setReference(ConvertArmExtendInchesToRotations(_ArmExtendSetpoint));
    }

    public void DecrementArmExtend()
    {
        _ArmExtendSetpoint -= 1;
        if (_ArmExtendSetpoint < 0)
            _ArmExtendSetpoint = 0;
        //convert from inches to motor rotations
        _ArmExtendPidRamp.setReference(ConvertArmExtendInchesToRotations(_ArmExtendSetpoint));
    }
    public void ArmExtendStop()
    {
        _ArmExtendPidRamp.Stop();
    }

    public void ArmExtendToPosition(double angle)
    {
        _ArmExtendSetpoint = angle;
        if (_ArmExtendSetpoint > ElevatorAndArmConstants.ArmExtendMax)
            _ArmExtendSetpoint = ElevatorAndArmConstants.ArmExtendMax;
        if (_ArmExtendSetpoint < 0)
            _ArmExtendSetpoint = 0;
        //convert to motor rotations
        _ArmExtendPidRamp.setReference(ConvertArmExtendInchesToRotations(_ArmExtendSetpoint));
    }

    /**
     * @param dist Inches
     * @return Rotations
     */
    private double ConvertArmExtendInchesToRotations(double dist)
    {
        return dist / (Math.PI * ElevatorAndArmConstants.ArmExtendSpoolDiameter) * ElevatorAndArmConstants.ArmExtendGearRatio;
    }
}
