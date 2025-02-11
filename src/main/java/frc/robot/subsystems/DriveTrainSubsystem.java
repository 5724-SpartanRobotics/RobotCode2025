package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Subsystems.Constant.CanIdConstants;

public class DriveTrainSubsystem {
    // Swerve modules
    private boolean parkFlag;
    private boolean doneFlag;

    private final SwerveModule LF;
    private final SwerveModule RF;
    private final SwerveModule LB;
    private final SwerveModule RB;
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDriveOdometry swerveDriveOdometry;
    private Pose2d robotPose;
    private final SwerveModule[] modules;
    private Rotation2d lastUpdatedGyroHeading;

    public class DriveTrainSubsystem extends SubsystemBase implements DriveTrainInterface { {    
        gyro = new Pigeon2(CanIdConstants.PigeonID);
        resetGyro();
        
        LF = new SwerveModule(CanIdConstants.LFTurnMotor, CanIdConstants.LFDriveMotor, CanIdConstants.LFCanID, DriveConstants.LFOff, "LF", this);
        RF = new SwerveModule(CanIdConstants.RFTurnMotor, CanIdConstants.RFDriveMotor, CanIdConstants.RFCanID, DriveConstants.RFOff, "RF", this);
        LB = new SwerveModule(CanIdConstants.LBTurnMotor, CanIdConstants.LBDriveMotor, CanIdConstants.LBCanID, DriveConstants.LBOff, "LB", this);
        RB = new SwerveModule(CanIdConstants.RBTurnMotor, CanIdConstants.RBDriveMotor, CanIdConstants.RBCanID, DriveConstants.RBOff, "RB", this);
        
        swerveDriveKinematics = new SwerveDriveKinematics(
            DriveConstants.LFLocation,
            DriveConstants.RFLocation,
            DriveConstants.LBLocation,
            DriveConstants.RBLocation
        );

        robotPose = new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d());
        
        SwerveModulePosition[] swerveInitialPositions = {
            LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()
        };

        swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroHeading(), swerveInitialPositions, robotPose);
        modules = new SwerveModule[] {LF, RF, LB, RB};
        
        parkFlag = false;
        doneFlag = false;
    }

    public Rotation2d getGyroHeading() {
        return lastUpdatedGyroHeading;
    }

    public void resetGyro() {
        gyro.reset();
        lastUpdatedGyroHeading = Rotation2d.fromDegrees(-gyro.getAngle());
    }

    @Override
    public void periodic() {
        lastUpdatedGyroHeading = Rotation2d.fromDegrees(-gyro.getAngle());
        robotPose = swerveDriveOdometry.update(getGyroHeading(), new SwerveModulePosition[] {
            LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()
        });

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("Gyro Heading Deg", getGyroHeading().getDegrees());
            SmartDashboard.putNumber("RobotPoseX", robotPose.getX());
            SmartDashboard.putNumber("RobotPoseY", robotPose.getY());
        }
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModStates = swerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading())
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);
        
        LF.setDesiredState(swerveModStates[0]);
        RF.setDesiredState(swerveModStates[1]);
        LB.setDesiredState(swerveModStates[2]);
        RB.setDesiredState(swerveModStates[3]);
    }
}

}