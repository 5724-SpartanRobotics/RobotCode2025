package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.InterferenceInfo;
import frc.robot.RobotContainer;
import frc.robot.lib.PidRamp;

public class ElevatorAndArmSubSys extends SubsystemBase {
    private SparkMax _ElevatorMtrCtrl1;
    private SparkMax _ElevatorMtrCtrl2;
    private SparkMax _ArmRotateMtrCtrl1;
    private SparkMax _ArmRotateMtrCtrl2;
    private SparkMax _ArmExtendMtrCtrl;
    private SparkMax _ClawIntake;

    private SparkClosedLoopController _ElevatorMtr1PidController;
    private RelativeEncoder _ElevatorMtr1Encoder;
    private SparkClosedLoopController _ArmRotateMtr1PidController;
    private RelativeEncoder _ArmRotateMtr1Encoder;
    private SparkClosedLoopController _ArmExtendMtrPidController;
    private RelativeEncoder _ArmExtenMtrEncoder;

    private final LedSubsystem _LedSubsystem;
    private PidRamp _ElevatorPidRamp;
    private PidRamp _ArmRotatePidRamp;
    private PidRamp _ArmExtendPidRamp;

    private double _ElevatorSetpoint;
    private double _ElevatorMax = 14.0;
    private double _ArmRotateSetpoint;
    private double _ArmRotateMax = 100;
    private double _ArmExtendSetpoint;
    private double _ArmExtendMax = 12.0;

    public ElevatorAndArmSubSys(LedSubsystem led)
    {
        _ElevatorMtrCtrl1 = new SparkMax(CanIdConstants.ElevatorMtrCtrl1CanId, MotorType.kBrushless);
        _ElevatorMtrCtrl2 = new SparkMax(CanIdConstants.ElevatorMtrCtrl2CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl1 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl1CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl2 = new SparkMax(CanIdConstants.ArmRotateMtrCtrl2CanId, MotorType.kBrushless);
        _ArmExtendMtrCtrl = new SparkMax(CanIdConstants.ArmExtendMtrCtrlCanId, MotorType.kBrushless);
        _ClawIntake = new SparkMax(CanIdConstants.ClawMtrCtrlCanId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ElePidP, ElevatorAndArmConstants.ElePidI, ElevatorAndArmConstants.ElePidD)
        .velocityFF(ElevatorAndArmConstants.ElePidFF);

        _ElevatorMtrCtrl1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
        cfg//.inverted(false)
        .idleMode(IdleMode.kBrake)
        .follow(CanIdConstants.ElevatorMtrCtrl1CanId, true);
        // cfg.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(ElevatorAndArmConstants.ElePidP, ElevatorAndArmConstants.ElePidI, ElevatorAndArmConstants.ElePidD)
        // .velocityFF(ElevatorAndArmConstants.ElePidFF);
        _ElevatorMtrCtrl2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
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

        _ClawIntake.configure(new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit(50)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmExtendPidP, ElevatorAndArmConstants.ArmExtendPidI, ElevatorAndArmConstants.ArmExtendPidD)
        .velocityFF(ElevatorAndArmConstants.ArmExtendPidFF);
        _ArmExtendMtrCtrl.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ElevatorMtr1PidController = _ElevatorMtrCtrl1.getClosedLoopController();
        _ElevatorMtr1Encoder = _ElevatorMtrCtrl1.getEncoder();
        _ArmRotateMtr1PidController = _ArmRotateMtrCtrl1.getClosedLoopController();
        _ArmRotateMtr1Encoder = _ArmRotateMtrCtrl1.getEncoder();
        _ArmExtendMtrPidController = _ArmExtendMtrCtrl.getClosedLoopController();
        _ArmExtenMtrEncoder = _ArmExtendMtrCtrl.getEncoder();
        _ElevatorPidRamp = new PidRamp(_ElevatorMtr1PidController, ConvertElevatorInchesToNeoRotations(ElevatorAndArmConstants.ElevatorSetpointRampRate));
        _ArmRotatePidRamp = new PidRamp(_ArmRotateMtr1PidController, ConvertArmRotateAngleToNeoRotations(ElevatorAndArmConstants.ArmRotateSetpointRampRate));
        _ArmExtendPidRamp = new PidRamp(_ArmExtendMtrPidController, ConvertArmExtendInchesToRotations(ElevatorAndArmConstants.ArmExtendSetpointRampRate));

        _LedSubsystem = led;
    }

    @Override
    public void periodic(){
        super.periodic();
        double armExtendPosition = GetArmExtendPosnInches();
        double armRotatePosition = GetArmRotateAngleDegrees();
        double elevatorPosition = GetElevatorHeightInches();

        _ElevatorPidRamp.Periodic(ConvertElevatorInchesToNeoRotations(elevatorPosition));
        _ArmRotatePidRamp.Periodic(ConvertArmRotateAngleToNeoRotations(armRotatePosition));
        _ArmExtendPidRamp.Periodic(ConvertArmExtendInchesToRotations(armExtendPosition));

        SmartDashboard.putNumber("ArmExtPos", armExtendPosition);
        SmartDashboard.putNumber("ArmRotPos", armRotatePosition);
        SmartDashboard.putNumber("ArmExtRef", _ArmExtendSetpoint);
        SmartDashboard.putNumber("ElevatorPos", elevatorPosition);
        SmartDashboard.putNumber("ElevatorPosRef", _ElevatorSetpoint);
        SmartDashboard.putNumber("ArmRotRef", _ArmRotateSetpoint);

        if (ClawIsUsingLotsOfCurrent()) {
            _LedSubsystem.setColor(LedSubsystem.kDefaultActiveColor);
        } else {
            _LedSubsystem.reset();
        }
    }

    public void MoveToL4()
    {
        
    }

    public void MoveToL3()
    {

    }

    public void MoveToL2()
    {

    }

    public void MoveToL1()
    {
        
    }

    public boolean ClawIsUsingLotsOfCurrent() {
        return _ClawIntake.getOutputCurrent() > 15.0;
    }

    public double GetElevatorHeightInches()
    {
        return _ElevatorMtr1Encoder.getPosition() / ElevatorAndArmConstants.ElevatorGearRatio * Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter;
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
        return (Math.cos(Math.toRadians(armAngleToHorizontal))/armCurrentLength) - ElevatorAndArmConstants.ArmPivotPointToFrameEdgeInches;
    }

    public void IncrementElevatorUp()
    {
        _ElevatorSetpoint += 1;
        if (_ElevatorSetpoint > _ElevatorMax)
            _ElevatorSetpoint = _ElevatorMax;
        _ElevatorPidRamp.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint));
    }

    /**
     * @param height Inches
     * @return Rotations
     */
    private double ConvertElevatorInchesToNeoRotations(double height)
    {
        return height / (Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter) * ElevatorAndArmConstants.ElevatorGearRatio;
    }

    public void DecrementElevatorUp()
    {
        _ElevatorSetpoint -= 1;//1 inch
        if (_ElevatorSetpoint < 0)
            _ElevatorSetpoint = 0;
        _ElevatorPidRamp.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint));
    }

    public void ElevatorStop()
    {
        _ElevatorPidRamp.Stop();
    }

    public void ElevatorToPosition(double heightInches)
    {
        _ElevatorSetpoint = heightInches;
        if (_ElevatorSetpoint > _ElevatorMax)
            _ElevatorSetpoint = _ElevatorMax;
        _ElevatorPidRamp.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint));
    }

    private double ConvertArmRotateAngleToNeoRotations(double angle)
    {
        return angle / 360 * ElevatorAndArmConstants.ArmRotateGearRatio;
    }
    public void IncrementArmRotate()
    {
        InterferenceInfo info = new InterferenceInfo();
        if (RobotContainer.InterferenceHelper.ArmCannotRotateUp(info))
        {
            SmartDashboard.putString("InterferenceMessage", info.Message);
            return;
        }
        else
        {
            SmartDashboard.putString("InterferenceMessage", "");
        }
        _ArmRotateSetpoint += 5;
        if (_ArmRotateSetpoint > _ArmRotateMax)
            _ArmRotateSetpoint = _ArmRotateMax;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }


    public void DecrementArmRotate()
    {
        InterferenceInfo info = new InterferenceInfo();
        if (RobotContainer.InterferenceHelper.ArmCannotRotateDown(info))
        {
            SmartDashboard.putString("InterferenceMessage", info.Message);
            return;
        }
        else
        {
            SmartDashboard.putString("InterferenceMessage", "");
        }
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
        if (_ArmRotateSetpoint > _ArmRotateMax)
            _ArmRotateSetpoint = _ArmRotateMax;
        if (_ArmRotateSetpoint < 0)
            _ArmRotateSetpoint = 0;
        //convert to motor rotations
        double setpoint = ConvertArmRotateAngleToNeoRotations(_ArmRotateSetpoint);
        _ArmRotatePidRamp.setReference(setpoint);
    }

    public void IncrementArmExtend()
    {
        _ArmExtendSetpoint += 1;
        if (_ArmExtendSetpoint > _ArmExtendMax)
            _ArmExtendSetpoint = _ArmExtendMax;
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
        if (_ArmExtendSetpoint > _ArmExtendMax)
            _ArmExtendSetpoint = _ArmExtendMax;
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

    public void ClawRun(double speed)
    {
        SmartDashboard.putNumber("ClawSpeed", speed);
        _ClawIntake.set(speed);
    }
}
