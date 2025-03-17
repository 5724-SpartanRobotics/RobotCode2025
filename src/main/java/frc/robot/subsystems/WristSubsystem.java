package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

public class WristSubsystem extends SubsystemBase implements PidEnabledSubsystemInterface {
    private SparkFlex _WristMtrCtrl;
    private SparkClosedLoopController _WristMtrPidController;
    private RelativeEncoder _WristMtrEncoder;
    private PidRamp _WristPidRamp;
    private double _WristSetpoint;

    public WristSubsystem()
    {
        _WristMtrCtrl = new SparkFlex(CanIdConstants.WristMtrCtrlCanId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(true)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorAndArmConstants.WristPidP, ElevatorAndArmConstants.WristPidI, ElevatorAndArmConstants.WristPidD)
        .velocityFF(ElevatorAndArmConstants.WristPidFF)
        .iMaxAccum(0.2);

        _WristMtrCtrl.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        _WristMtrPidController = _WristMtrCtrl.getClosedLoopController();
        _WristMtrEncoder = _WristMtrCtrl.getEncoder();
        _WristPidRamp = new PidRamp(_WristMtrPidController, ConvertWristDegreesToMotorRotations(ElevatorAndArmConstants.WristSetpointRampRate));

    }

    @Override
    public void periodic() {
        super.periodic();
        double wristPosition = GetWristDegrees();
        _WristPidRamp.Periodic(ConvertWristDegreesToMotorRotations(wristPosition));
         if (DebugSetting.TraceLevel == DebugLevel.Wrist || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber("WristPos", wristPosition);
            SmartDashboard.putNumber("WristRef", _WristSetpoint);
            SmartDashboard.putNumber("WristRameOutRef", _WristPidRamp.GetCurrentRampedSetpoint());
        }
    }

    public void resetReferences() {
        WristToPosition(0);
    }

    private double ConvertWristDegreesToMotorRotations(double angle){
        return angle / 360 * ElevatorAndArmConstants.WristGearRatio;
    }

    public double GetWristDegrees() {
        return _WristMtrEncoder.getPosition() / ElevatorAndArmConstants.WristGearRatio * 360;
    }

    public void IncrementWrist() {
        _WristSetpoint += 1;
        if (_WristSetpoint > ElevatorAndArmConstants.WristMax)
            _WristSetpoint = ElevatorAndArmConstants.WristMax;
        //convert from inches to motor rotations
        _WristPidRamp.setReference(ConvertWristDegreesToMotorRotations(_WristSetpoint));
    }

    public void DecrementWrist()
    {
        _WristSetpoint -= 1;
        if (_WristSetpoint < 0)
            _WristSetpoint = 0;
        //convert from inches to motor rotations
        _WristPidRamp.setReference(ConvertWristDegreesToMotorRotations(_WristSetpoint));
    }
    public void WristStop()
    {
        _WristPidRamp.Stop();
    }

    public void WristToPosition(double angle)
    {
        _WristSetpoint = angle;
        if (_WristSetpoint > ElevatorAndArmConstants.WristMax)
            _WristSetpoint = ElevatorAndArmConstants.WristMax;
        if (_WristSetpoint < 0)
            _WristSetpoint = 0;
        //convert to motor rotations
        _WristPidRamp.setReference(ConvertWristDegreesToMotorRotations(_WristSetpoint));
    }

}
