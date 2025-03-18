package frc.robot.lib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;
import frc.robot.Constants.DriveConstants;
import frc.robot.Util.CTREModuleState;
import frc.robot.Util.Conversions;

public class SwerveModule {
    private SparkFlex turn;
    private SparkBaseConfig motorConfig;
    private SparkFlex drive;
    private SparkClosedLoopController turn_pid;
    private CANcoder canCoder;

    private double driveSpeed = 0;
    private double driveAngle = 0;
    private double isInverted = 1;

    public String Name;
    private double Offset;

    public SwerveModule(int turnMotor, int driveMotor, int canCoderID, double offset, boolean inverted, String name) {
        this.drive = new SparkFlex(driveMotor, MotorType.kBrushless);
        this.turn = new SparkFlex(turnMotor, MotorType.kBrushless);
        this.motorConfig = new SparkFlexConfig()
            .inverted(false)
            .apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false))
            .apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false))
            .apply(new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1).i(0.0).d(0.0)
            )
            .idleMode(IdleMode.kBrake);

        this.drive.configure(this.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.turn.configure(this.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.isInverted = inverted ? -1 : 1;


        this.Offset = offset;
        this.canCoder = new CANcoder(canCoderID);
        this.turn_pid = this.turn.getClosedLoopController();
        resetEncoders();
        // lastAngle = getState().angle;

        this.Name = name;
        // this.canCoderName = name + canCoderID;
        resetTurnToAbsolute();
        applyTurnConfiguration();
    }

    public void setCartesian(double x, double y) {
        // requestedAngle = Math.atan2(y, x);
        driveSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void setPolar(double ang, double spe) {
        // requestedAngle = ang;
        driveSpeed = spe;
    }



    public SwerveModuleState getState() {
        double velocity = drive.get();
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.vortexToDegrees(-turn.getEncoder().getPosition()));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = Conversions.vortexToMeters(drive.getEncoder().getPosition());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.vortexToDegrees(-turn.getEncoder().getPosition()));
        return new SwerveModulePosition(position, angle);
    }

    public void reportAbsolutePosition() {
        SmartDashboard.putNumber(Name + " AbsEncPosn", canCoder.getAbsolutePosition().refresh().getValueAsDouble());
    }

    public void reportMotorEncoderPosition() {
        SmartDashboard.putNumber(Name + " TurnMotorPosEnc", turn.getEncoder().getPosition()); // rotations
    }

    public void reportMotorCurrent() {
        SmartDashboard.putNumber(Name + "_DriveCurrent", drive.getOutputCurrent());
        SmartDashboard.putNumber(Name + "_TurnCurrent", turn.getOutputCurrent());
    }

    public void reportAll() {
        reportAbsolutePosition();
        reportMotorEncoderPosition();
        reportMotorCurrent();
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        driveSpeed = desiredState.speedMetersPerSecond / DriveConstants.maxRobotSpeedmps;
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " DriveRef", driveSpeed);
        }
        drive.set(driveSpeed * isInverted);

        //if desired speed is less than 1 percent, keep the angle where it was to prevent jittering
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxRobotSpeedmps * 0.01)) ? driveAngle : desiredState.angle.getRadians();
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " TurnRef", Units.radiansToDegrees(angle));
        }
        turn_pid.setReference(-Conversions.radiansToVortex(angle), ControlType.kPosition);
        driveAngle = angle;
    }



    public void resetTurnToAbsolute() {
        double absPos = canCoder.getAbsolutePosition().refresh().getValueAsDouble() * Constants.TwoPI; // radians
        double absolutePosition = Conversions.radiansToVortex(absPos - Offset);
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber(Name + " Init Posn abs", absolutePosition);
        }
        
        turn.getEncoder().setPosition(-absolutePosition);
    }

    public void applyTurnConfiguration() {
        REVLibError status;
        for (int i = 0; i < 5; i++) {
            status = turn.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            if (status.equals(REVLibError.kOk)) break;
            else SmartDashboard.putString(Name + " Config Error", status.toString());
        }
    }

    public void resetEncoders() {
        drive.getEncoder().setPosition(0);
        double absoluteEncoderAngle = (canCoder.getAbsolutePosition().refresh().getValueAsDouble() - Offset) * Constants.TwoPI * 1.0 /* encoder not reversed */;
        turn.getEncoder().setPosition(absoluteEncoderAngle / (Constants.TwoPI / DriveConstants.turnGearRatio));
    }
}
 
