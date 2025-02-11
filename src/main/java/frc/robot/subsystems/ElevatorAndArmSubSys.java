package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.ElevatorAndArmConstants;

public class ElevatorAndArmSubSys extends SubsystemBase{

    private SparkMax _ElevatorMtrCtrl1;
    private SparkMax _ElevatorMtrCtrl2;
    private SparkMax _ArmRotateMtrCtrl1;
    private SparkMax _ArmRotateMtrCtrl2;
    private SparkMax _ArmExtendMtrCtrl;

    private SparkClosedLoopController _ElevatorMtr1PidController;
    private RelativeEncoder _ElevatorMtr1Encoder;
    private SparkClosedLoopController _ArmRotateMtr1PidController;
    private RelativeEncoder _ArmRotateMtr1Encoder;
    private SparkClosedLoopController _ArmExtendMtrPidController;
    private RelativeEncoder _ArmExtenMtrEncoder;

    public ElevatorAndArmSubSys()
    {
        _ElevatorMtrCtrl1 = new SparkMax(ElevatorAndArmConstants.ElevatorMtrCtrl1CanId, MotorType.kBrushless);
        _ElevatorMtrCtrl2 = new SparkMax(ElevatorAndArmConstants.ElevatorMtrCtrl2CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl1 = new SparkMax(ElevatorAndArmConstants.ArmRotateMtrCtrl1CanId, MotorType.kBrushless);
        _ArmRotateMtrCtrl2 = new SparkMax(ElevatorAndArmConstants.ArmRotateMtrCtrl2CanId, MotorType.kBrushless);
        _ArmExtendMtrCtrl = new SparkMax(ElevatorAndArmConstants.ArmExtendMtrCtrlCanId, MotorType.kBrushless);

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
        follow(ElevatorAndArmConstants.ElevatorMtrCtrl1CanId);
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
        .follow(ElevatorAndArmConstants.ArmRotateMtrCtrl1CanId);
        _ArmRotateMtrCtrl2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ArmExtendPidP, ElevatorAndArmConstants.ArmExtendPidI, ElevatorAndArmConstants.ArmExtendPidD)
        .velocityFF(ElevatorAndArmConstants.ArmExtendPidFF);
        _ArmRotateMtrCtrl1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ElevatorMtr1PidController = _ElevatorMtrCtrl1.getClosedLoopController();
        _ElevatorMtr1Encoder = _ElevatorMtrCtrl1.getEncoder();
        _ArmRotateMtr1PidController = _ArmRotateMtrCtrl1.getClosedLoopController();
        _ArmRotateMtr1Encoder = _ArmRotateMtrCtrl1.getEncoder();
        _ArmExtendMtrPidController = _ArmExtendMtrCtrl.getClosedLoopController();
        _ArmExtenMtrEncoder = _ArmExtendMtrCtrl.getEncoder();
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
    }

    public void MoveToPickupPieceInRobot()
    {
        _ArmExtendMtrPidController.setReference(0, ControlType.kPosition);
        _ArmRotateMtr1PidController.setReference(0, ControlType.kPosition);
        _ElevatorMtr1PidController.setReference(0, ControlType.kPosition);
    }
}
