package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;

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

public class ElevatorSubsystem extends SubsystemBase implements PidEnabledSubsystemInterface {
    private SparkMax _ElevatorMtrCtrl1;
    private SparkMax _ElevatorMtrCtrl2;
    private SparkClosedLoopController _ElevatorMtr1PidController;
    private RelativeEncoder _ElevatorMtr1Encoder;
    private SparkClosedLoopController _ElevatorMtr2PidController;
    private RelativeEncoder _ElevatorMtr2Encoder;
    private PidRamp _ElevatorPidRamp;
    private double _ElevatorSetpoint;
    private Instant _DateTimeOfLastElevatorStall = Instant.MIN;
    private double _ElevatorPosnLastStallCheck;
    

    public ElevatorSubsystem() {
        _ElevatorMtrCtrl1 = new SparkMax(CanIdConstants.ElevatorMtrCtrl1CanId, MotorType.kBrushless);
        _ElevatorMtrCtrl2 = new SparkMax(CanIdConstants.ElevatorMtrCtrl2CanId, MotorType.kBrushless);
        

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
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.ElePidP, ElevatorAndArmConstants.ElePidI, ElevatorAndArmConstants.ElePidD)
        .velocityFF(ElevatorAndArmConstants.ElePidFF);
        _ElevatorMtrCtrl2.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        _ElevatorMtr1PidController = _ElevatorMtrCtrl1.getClosedLoopController();
        _ElevatorMtr1Encoder = _ElevatorMtrCtrl1.getEncoder();
        _ElevatorMtr2PidController = _ElevatorMtrCtrl2.getClosedLoopController();
        _ElevatorMtr2Encoder = _ElevatorMtrCtrl2.getEncoder();
        _ElevatorPidRamp = new PidRamp(_ElevatorMtr1PidController, _ElevatorMtr2PidController, ConvertElevatorInchesToNeoRotations(ElevatorAndArmConstants.ElevatorSetpointRampRate));

    }

    @Override
    public void periodic() {
        super.periodic();

        double elevatorPosition = GetElevatorHeightInches();
        _ElevatorPidRamp.Periodic(ConvertElevatorInchesToNeoRotations(elevatorPosition));

        if (DebugSetting.TraceLevel == DebugLevel.Elevator || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("ElevatorPos", elevatorPosition);
            SmartDashboard.putNumber("ElevatorPosRef", _ElevatorSetpoint);
            SmartDashboard.putNumber("ElevatorPosM2", GetElevatorHeightInches(true));
        }
        SmartDashboard.putBoolean("ElevatorStalled", GetElevatorLikelySufferedChainJump());

        //reset the elevator position to zero to avoid motor / controller burn up.
        if (GetElevatorLikelySufferedChainJump())
            _ElevatorMtr1Encoder.setPosition(0);
    }

    public void resetReferences() {
        ElevatorToPosition(0);
    }

    public double GetElevatorHeightInches(Boolean motor2) {
        return (motor2 == null || !motor2.booleanValue()) ? _ElevatorMtr1Encoder.getPosition() / ElevatorAndArmConstants.ElevatorGearRatio * Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter :
            _ElevatorMtr2Encoder.getPosition() / ElevatorAndArmConstants.ElevatorGearRatio * Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter;
    }

    public double GetElevatorHeightInches() {
        return _ElevatorMtr1Encoder.getPosition() / ElevatorAndArmConstants.ElevatorGearRatio * Math.PI * ElevatorAndArmConstants.ElevatorChainSproketDiameter;
    }

    /**
     * if the elevator chain slips (which would happen only on raising), then when it returns to zero
     * height, the position regulator will continue to attempt to drive it lower against the stops.
     * We will detect this by looking for:
     *  - Ramped position reference is zero 
     *  - The absolute value of motor current is above some set value (emperical stall current is 105 Amps)
     *  - The position is not near zero and is not changing.
     * The above 3 conditions have been in effect for 3 seconds.
     * @return
     */
    private boolean GetElevatorLikelySufferedChainJump()
    {
        double elevatorPosn = GetElevatorHeightInches();
        boolean stallLikely = _ElevatorPidRamp.GetCurrentRampedSetpoint() == 0 &&
        Math.abs(_ElevatorMtrCtrl1.getOutputCurrent()) > 70.0 &&
        Math.abs(elevatorPosn - _ElevatorPosnLastStallCheck) <= 0.1 &&//indication of not moving
        elevatorPosn > 0.75;
        _ElevatorPosnLastStallCheck = elevatorPosn;
        if (_DateTimeOfLastElevatorStall == Instant.MIN && stallLikely)
        {
            _DateTimeOfLastElevatorStall = Instant.now();
        }
        else if (!stallLikely)
        {
            _DateTimeOfLastElevatorStall = Instant.MIN;
        }
        if (_DateTimeOfLastElevatorStall != Instant.MIN && 
        Duration.between(_DateTimeOfLastElevatorStall, Instant.now()).compareTo(Duration.ofSeconds(3)) > 0)
            return true;
        return false;
    }
    public void IncrementElevatorUp()
    {
        _ElevatorSetpoint += 1;
        if (_ElevatorSetpoint > ElevatorAndArmConstants.ElevatorMax)
            _ElevatorSetpoint = ElevatorAndArmConstants.ElevatorMax;
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
        if (_ElevatorSetpoint > ElevatorAndArmConstants.ElevatorMax)
            _ElevatorSetpoint = ElevatorAndArmConstants.ElevatorMax;
        _ElevatorPidRamp.setReference(ConvertElevatorInchesToNeoRotations(_ElevatorSetpoint));
    }

}
