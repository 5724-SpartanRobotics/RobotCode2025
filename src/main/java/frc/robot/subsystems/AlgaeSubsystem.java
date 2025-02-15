package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.NeoConstants;

public class AlgaeSubsystem extends SubsystemBase{
    private SparkMax _AlgaeRotateMtrCtrl;
    private SparkClosedLoopController _AlgaeRotateMtrPidController;
    private RelativeEncoder _AlgaeRotateMtrEncoder;

    private double _AlgaeRotateSetpoint;

    public AlgaeSubsystem()
    {
        _AlgaeRotateMtrCtrl = new SparkMax(CanIdConstants.AlgaeRotateMtrCtrlCanId, MotorType.kBrushless);
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(false)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeConstants.RotatePidP, AlgaeConstants.RotatePidI, AlgaeConstants.RotatePidD)
        .velocityFF(AlgaeConstants.RotatePidFF);

        _AlgaeRotateMtrCtrl.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        _AlgaeRotateMtrPidController = _AlgaeRotateMtrCtrl.getClosedLoopController();
        _AlgaeRotateMtrEncoder = _AlgaeRotateMtrCtrl.getEncoder();
    }
    @Override
    public void periodic(){
        super.periodic();
        double algaeRotatePosn = _AlgaeRotateMtrEncoder.getPosition();

        SmartDashboard.putNumber("AlgaePos", algaeRotatePosn);
    }

    public void RotateToOut()
    {
        double degreesOut = 110.0;
        _AlgaeRotateSetpoint = degreesOut;
        double setpoint = degreesOut * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void RotateToIn()
    {
        _AlgaeRotateMtrPidController.setReference(0, ControlType.kPosition);
    }

    public void IncrementSmallDegrees()
    {
        double newSetpoint = _AlgaeRotateSetpoint + AlgaeConstants.IncrementDegrees;
        if (newSetpoint > 120)
            newSetpoint = 120;
        _AlgaeRotateSetpoint = newSetpoint;
        double setpoint = _AlgaeRotateSetpoint * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void DecrementSmallDegrees()
    {
        double newSetpoint = _AlgaeRotateSetpoint - AlgaeConstants.IncrementDegrees;
        if (newSetpoint < 0)
            newSetpoint = 0;
        _AlgaeRotateSetpoint = newSetpoint;
        double setpoint = _AlgaeRotateSetpoint * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }
}
