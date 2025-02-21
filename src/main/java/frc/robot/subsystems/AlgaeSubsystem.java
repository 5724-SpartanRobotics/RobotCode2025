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
import frc.robot.lib.PidRamp;

public class AlgaeSubsystem extends SubsystemBase{
    private SparkMax _AlgaeRotateMtrCtrl;
    private SparkClosedLoopController _AlgaeRotateMtrPidController;
    private RelativeEncoder _AlgaeRotateMtrEncoder;
    private double _AlgaeRotateMax = 120;
    private PidRamp _PidRamp;

    private double _AlgaeRotateSetpoint;

    public AlgaeSubsystem()
    {
        _AlgaeRotateMtrCtrl = new SparkMax(CanIdConstants.AlgaeRotateMtrCtrlCanId, MotorType.kBrushless);
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.inverted(false)
        .idleMode(IdleMode.kBrake);
        cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(AlgaeConstants.RotatePidP, AlgaeConstants.RotatePidI, AlgaeConstants.RotatePidD, AlgaeConstants.RotatePidFF);
     
        _AlgaeRotateMtrCtrl.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        _AlgaeRotateMtrPidController = _AlgaeRotateMtrCtrl.getClosedLoopController();
        _AlgaeRotateMtrEncoder = _AlgaeRotateMtrCtrl.getEncoder();
        _PidRamp = new PidRamp(_AlgaeRotateMtrPidController, (AlgaeConstants.AlgaeSetpointRampRate * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0));
    }
    @Override
    public void periodic(){
        super.periodic();
        double algaeRotatePosn = GetAlgaeArmAngleDegrees();
        double motorPosn = algaeRotatePosn * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360;
        _PidRamp.Periodic(motorPosn);

        SmartDashboard.putNumber("AlgaePos", algaeRotatePosn);
        SmartDashboard.putNumber("AlgaeRef", _AlgaeRotateSetpoint);
        SmartDashboard.putNumber("AlgaeRampRef", _PidRamp.GetCurrentRampedSetpoint());
        SmartDashboard.putBoolean("AlgaeStopActive", _PidRamp.GetStopIsActive());
    }

        /**
     * Call this at teleop init, so if we disabled with a setpoint and
     * an arm or elevator drifted, we can set the motor controller's PID setpoint
     * to match the current state.
     */
    public void SetPositionRegulatorsSetpointToMatchFeedback() {
        _AlgaeRotateMtrPidController.setReference(_AlgaeRotateMtrEncoder.getPosition(), ControlType.kPosition);
    }

    public void RotateToOut()
    {
        double degreesOut = AlgaeConstants.AlgaeFullOutAngle;
        _AlgaeRotateSetpoint = degreesOut;
        double setpoint = degreesOut * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _PidRamp.setReference(setpoint);
//        _AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void RotateToIn()
    {
        _AlgaeRotateSetpoint = 0;
        _PidRamp.setReference(_AlgaeRotateSetpoint);
        //_AlgaeRotateMtrPidController.setReference(_AlgaeRotateSetpoint, ControlType.kPosition);
    }

    public void RotateToHoldAlgaePosn()
    {
        double degrees = AlgaeConstants.AlgaeHoldGamepieceAngle;
        _AlgaeRotateSetpoint = degrees;
        double setpoint = degrees * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _PidRamp.setReference(setpoint);
        //_AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void AlgaeToSetpoint(double angle)
    {
        _AlgaeRotateSetpoint = angle;
        if (_AlgaeRotateSetpoint > _AlgaeRotateMax)
            _AlgaeRotateSetpoint = _AlgaeRotateMax;
        if (_AlgaeRotateSetpoint < 0)
            _AlgaeRotateSetpoint = 0;
        double setpoint = _AlgaeRotateSetpoint * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _PidRamp.setReference(setpoint);
        //_AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void AlgaeStop()
    {
        _PidRamp.Stop();
        //_AlgaeRotateMtrPidController.setReference(0, ControlType.kVelocity);
    }

    public void IncrementSmallDegrees()
    {
        double newSetpoint = _AlgaeRotateSetpoint + AlgaeConstants.IncrementDegrees;
        if (newSetpoint > _AlgaeRotateMax)
            newSetpoint = _AlgaeRotateMax;
        _AlgaeRotateSetpoint = newSetpoint;
        double setpoint = _AlgaeRotateSetpoint * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _PidRamp.setReference(setpoint);
        //_AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void DecrementSmallDegrees()
    {
        double newSetpoint = _AlgaeRotateSetpoint - AlgaeConstants.IncrementDegrees;
        if (newSetpoint < 0)
            newSetpoint = 0;
        _AlgaeRotateSetpoint = newSetpoint;
        double setpoint = _AlgaeRotateSetpoint * NeoConstants.CountsPerRevolution * AlgaeConstants.GearRatio / 360.0;
        _PidRamp.setReference(setpoint);
        //_AlgaeRotateMtrPidController.setReference(setpoint, ControlType.kPosition);
    }

    public double GetAlgaeArmAngleDegrees()
    {
        double neoRotations = _AlgaeRotateMtrEncoder.getPosition();
        return neoRotations / AlgaeConstants.GearRatio * 360;
    }
}
