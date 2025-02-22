package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.lib.SwerveModule;

public class DriveTrainSubsystem extends SubsystemBase {
    public static final boolean kFieldRelativeDriveDefault = true;

    private final SwerveDriveOdometry _SwerveDriveOdometry;
    private final SwerveDriveKinematics _SwerveDriveKinematics;
    private final SwerveModule _LF;
    private final SwerveModule _RF;
    private final SwerveModule _LB;
    private final SwerveModule _RB;
    private final SwerveModule[] _SwerveModules;
    private final Pigeon2 _gyroscope;
    private final SwerveDrivePoseEstimator _SwerveDrivePoseEstimator;

    private Rotation2d lastUpdatedGyroHeading;
    private Pose2d robotPose;
    private Field2d field;

    private TeleopSwerve.JoystickAxes joystickAxes;

    public DriveTrainSubsystem() {
        _gyroscope = new Pigeon2(CanIdConstants.PigeonID);
        resetGyro();

        _LF = new SwerveModule(CanIdConstants.LFTurnMotor, CanIdConstants.LFDriveMotor, CanIdConstants.LFCanID, DriveConstants.LFOff, false, "LF");
        _RF = new SwerveModule(CanIdConstants.RFTurnMotor, CanIdConstants.RFDriveMotor, CanIdConstants.RFCanID, DriveConstants.RFOff, false, "RF");
        _LB = new SwerveModule(CanIdConstants.LBTurnMotor, CanIdConstants.LBDriveMotor, CanIdConstants.LBCanID, DriveConstants.LBOff, true, "LB");
        _RB = new SwerveModule(CanIdConstants.RBTurnMotor, CanIdConstants.RBDriveMotor, CanIdConstants.RBCanID, DriveConstants.RBOff, true, "RB");

        _SwerveDriveKinematics = new SwerveDriveKinematics(
            DriveConstants.LFLocation,
            DriveConstants.RFLocation,
            DriveConstants.LBLocation,
            DriveConstants.RBLocation
        );

        field = new Field2d();
        robotPose = new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d());

        SwerveModulePosition[] swerveInitialPositions = {
            _LF.getPosition(), _RF.getPosition(), _LB.getPosition(), _RB.getPosition()
        };

        _SwerveDriveOdometry = new SwerveDriveOdometry(_SwerveDriveKinematics, getGyroHeading(), swerveInitialPositions, robotPose);
        _SwerveModules = new SwerveModule[]{_LF, _RF, _LB, _RB};
        _SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(_SwerveDriveKinematics, getGyroHeading(), swerveInitialPositions, robotPose);
    }

    @Override
    public void periodic() {
        super.periodic();
        updateGyro();

        Rotation2d currentHdg = getGyroHeading();
        SwerveModulePosition[] positions = new SwerveModulePosition[]{
            _LF.getPosition(), _RF.getPosition(), _LB.getPosition(), _RB.getPosition()
        };
        robotPose = _SwerveDriveOdometry.update(currentHdg, positions);
        field.setRobotPose(robotPose);

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("RobotPoseX", robotPose.getX());
            SmartDashboard.putNumber("RobotPoseY", robotPose.getY());
            _LF.reportAll();
            _RF.reportAll();
            _LB.reportAll();
            _RB.reportAll();
            SmartDashboard.putNumber("GyroYaw", getGyroHeading().getDegrees());
            // _LF.periodic();
            // _RF.periodic();
            // _LB.periodic();
            // _RB.periodic();
        }

        _SwerveDrivePoseEstimator.update(currentHdg, positions);
    }

    public Rotation2d getGyroHeading() {
        return lastUpdatedGyroHeading;
    }

    public double getGyroRate() {
        return _gyroscope.getAngularVelocityZWorld().getValueAsDouble();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return _SwerveDrivePoseEstimator;
    }

    public DriveTrainSubsystem drive(Translation2d translation, double rotation) {
        SwerveModuleState[] states = _SwerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading())
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxRobotSpeedmps);

        _LF.setDesiredState(states[0]);
        _RF.setDesiredState(states[1]);
        _LB.setDesiredState(states[2]);
        _RB.setDesiredState(states[3]);

        return this;
    }

    public DriveTrainSubsystem brake() {
        for (SwerveModule m : _SwerveModules) m.setDesiredState(new SwerveModuleState(0, m.getState().angle));
        return this;
    }


    private void updateGyro() {
        lastUpdatedGyroHeading = Rotation2d.fromDegrees(_gyroscope.getYaw().getValueAsDouble());
    }


    private void resetGyro() {
        _gyroscope.reset();
        updateGyro();
    }

    public void zeroGyro() {
        _gyroscope.setYaw(0);
    }

    public void flipGyro() {
        _gyroscope.setYaw(Math.PI);
    }

    public TeleopSwerve.JoystickAxes setJoystickAxes(TeleopSwerve.JoystickAxes axes) {
        joystickAxes = axes;
        return joystickAxes;
    }
}