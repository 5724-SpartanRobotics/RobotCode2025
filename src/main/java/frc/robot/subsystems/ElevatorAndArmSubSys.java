package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InterferenceInfo;
import frc.robot.RobotContainer;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ElevatorAndArmConstants;

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
        cfg.inverted(false)
        .idleMode(IdleMode.kBrake).
        follow(CanIdConstants.ElevatorMtrCtrl1CanId);
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
        cfg.inverted(false)
        .idleMode(IdleMode.kBrake)
        .follow(CanIdConstants.ArmRotateMtrCtrl1CanId);
        _ArmRotateMtrCtrl2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmExtendPidP, ElevatorAndArmConstants.ArmExtendPidI, ElevatorAndArmConstants.ArmExtendPidD)
        .velocityFF(ElevatorAndArmConstants.ArmExtendPidFF);
        _ArmRotateMtrCtrl1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ClawIntake.configure(new SparkMaxConfig()
            .inverted(false)
            .smartCurrentLimit(50)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .idleMode(IdleMode.kBrake), 
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        _ElevatorMtr1PidController = _ElevatorMtrCtrl1.getClosedLoopController();
        _ElevatorMtr1Encoder = _ElevatorMtrCtrl1.getEncoder();
        _ArmRotateMtr1PidController = _ArmRotateMtrCtrl1.getClosedLoopController();
        _ArmRotateMtr1Encoder = _ArmRotateMtrCtrl1.getEncoder();
        _ArmExtendMtrPidController = _ArmExtendMtrCtrl.getClosedLoopController();
        _ArmExtenMtrEncoder = _ArmExtendMtrCtrl.getEncoder();

        _LedSubsystem = led;
    }

    @Override
    public void periodic(){
        super.periodic();
        double armExtendPosition = _ArmExtenMtrEncoder.getPosition();
        double armRotatePosition = _ArmRotateMtr1Encoder.getPosition();
        double elevatorPosition = _ElevatorMtr1Encoder.getPosition();

        SmartDashboard.putNumber("ArmExtPos", armExtendPosition);
        SmartDashboard.putNumber("ArmRotPos", armRotatePosition);
        SmartDashboard.putNumber("ElevatorPos", elevatorPosition);

        if (usingLotsOfCurrent()) {
            _LedSubsystem.setColor(Color.kGreen);
        } else {
            _LedSubsystem.reset();
        }
    }

    public void MoveToPickupPieceInRobot()
    {
        _ArmExtendMtrPidController.setReference(0, ControlType.kPosition);
        _ArmRotateMtr1PidController.setReference(0, ControlType.kPosition);
        _ElevatorMtr1PidController.setReference(0, ControlType.kPosition);
    }

    public boolean usingLotsOfCurrent() {
        return _ClawIntake.getOutputCurrent() > 30.0;
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
        _ElevatorMtr1PidController.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint), ControlType.kPosition);
    }

    private double ConvertElevatorInchesToNeoRotations(double height)
    {
        return height / (Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter) * ElevatorAndArmConstants.ElevatorGearRatio;
    }

    public void DecrementElevatorUp()
    {
        _ElevatorSetpoint -= 1;//1 inch
        if (_ElevatorSetpoint < 0)
            _ElevatorSetpoint = 0;
        _ElevatorMtr1PidController.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint), ControlType.kPosition);
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
        double setpoint = _ArmRotateSetpoint / 360 * ElevatorAndArmConstants.ArmRotateGearRatio;
        _ArmRotateMtr1PidController.setReference(setpoint, ControlType.kPosition);
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
        double setpoint = _ArmRotateSetpoint / 360 * ElevatorAndArmConstants.ArmRotateGearRatio;
        _ArmRotateMtr1PidController.setReference(setpoint, ControlType.kPosition);
    }

    public void IncrementArmExtend()
    {
        _ArmExtendSetpoint += 1;
        if (_ArmExtendSetpoint > _ArmExtendMax)
            _ArmExtendSetpoint = _ArmExtendMax;
        //convert from inches to motor rotations
        _ArmExtendMtrPidController.setReference(ConvertArmExtendInchesToRotations(_ArmExtendSetpoint), ControlType.kPosition);
    }

    public void DecrementArmExtend()
    {
        _ArmExtendSetpoint -= 1;
        if (_ArmExtendSetpoint < 0)
            _ArmExtendSetpoint = 0;
        //convert from inches to motor rotations
        _ArmExtendMtrPidController.setReference(ConvertArmExtendInchesToRotations(_ArmExtendSetpoint), ControlType.kPosition);
    }

    private double ConvertArmExtendInchesToRotations(double dist)
    {
        return dist / (Math.PI * ElevatorAndArmConstants.ArmExtendSpoolDiameter) * ElevatorAndArmConstants.ArmExtendGearRatio;
    }

    public void ClawRun(double speed)
    {
        _ClawIntake.set(speed);
    }
}
