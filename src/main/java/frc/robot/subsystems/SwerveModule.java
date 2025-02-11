package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Constant;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Util.CTREModuleState;
import frc.robot.Util.Conversions;

public class SwerveModule {

    private double offset = 0;

    private SparkFlex turn;
    private SparkFlexConfig motorConfig;
    private SparkFlex drive;
    private SparkClosedLoopController turn_pid;
    private CANcoder canCoder;
    private CANcoderConfiguration config;

    private DriveTrainInterface driveTrainParent;

    private SwerveModuleState state;

    private double driveSpeed = 0;
    private double driveAngle = 0;
    private double maxdrive = 0.3;
    private double isInverted = 1;
    private double requestedAngle = 0;
    private double canCoderModified = 0;

    private PIDController turnPID;
    private int turnID = 0;
    public String Name;
    private String canCoderName;

    public SwerveModule(int turnMotor, int driveMotor, int canCoderID, double off, String name, DriveTrainInterface driveTr) {
        Name = name;
        offset = off;
        turnID = turnMotor;
        driveTrainParent = driveTr;
        motorConfig = new SparkFlexConfig();

        turn = new SparkFlex(turnMotor, MotorType.kBrushless);
        drive = new SparkFlex(driveMotor, MotorType.kBrushless);
        turn.setInverted(false);
        drive.setInverted(false);

        canCoder = new CANcoder(canCoderID);
        canCoderName = name + canCoderID;
        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0);
        
       turn.configure(motorConfig, null, null);
       drive.configure(motorConfig, null, null);


    }

    public void update() {
        drive.set(maxdrive * isInverted * driveSpeed);

        double dashboardTurnSetpoint = SmartDashboard.getNumber("turn", 0);
        turn_pid.setReference(dashboardTurnSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("turn", dashboardTurnSetpoint);
    }

    public void setCartesian(double x, double y) {
        requestedAngle = Math.atan2(y, x);
        driveSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void setPolar(double ang, double spe) {
        requestedAngle = ang;
        driveSpeed = spe;
    }



    public SwerveModuleState getState() {
        double velocity = drive.get();
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(-turn.getEncoder().getPosition()));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = Conversions.falconToMeters(drive.getEncoder().getPosition());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(-turn.getEncoder().getPosition()));
        return new SwerveModulePosition(position, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        driveSpeed = desiredState.speedMetersPerSecond / DriveConstants.maxRobotSpeedmps;
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " DriveRef", driveSpeed);
        }
        drive.set(driveSpeed);

        //if desired speed is less than 1 percent, keep the angle where it was to prevent jittering
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxRobotSpeedmps * 0.01)) ? driveAngle : desiredState.angle.getRadians();
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All){
            SmartDashboard.putNumber(Name + " TurnRef", Units.radiansToDegrees(angle));
        }
        // turn.set(ControlMode.Position, -Conversions.radiansToFalcon(angle));
        turn_pid.setReference(-Conversions.radiansToFalcon(angle), ControlType.kPosition);
        driveAngle = angle;

    }

}
 
