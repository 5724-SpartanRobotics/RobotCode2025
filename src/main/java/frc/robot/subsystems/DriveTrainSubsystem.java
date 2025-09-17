package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    

    private Rotation2d lastUpdatedGyroHeading;
    private Pose2d robotPose;
    private Field2d field;

    private TeleopSwerve.JoystickAxes joystickAxes;

    public DriveTrainSubsystem() {
        _gyroscope = new Pigeon2(CanIdConstants.PigeonID);
        resetGyro();

        _LF = new SwerveModule(CanIdConstants.LFTurnMotor, CanIdConstants.LFDriveMotor, CanIdConstants.LFCanID, DriveConstants.LFOff, true, "LF");
        _RF = new SwerveModule(CanIdConstants.RFTurnMotor, CanIdConstants.RFDriveMotor, CanIdConstants.RFCanID, DriveConstants.RFOff, true, "RF");
        _LB = new SwerveModule(CanIdConstants.LBTurnMotor, CanIdConstants.LBDriveMotor, CanIdConstants.LBCanID, DriveConstants.LBOff, true, "LB");
        _RB = new SwerveModule(CanIdConstants.RBTurnMotor, CanIdConstants.RBDriveMotor, CanIdConstants.RBCanID, DriveConstants.RBOff, true, "RB");

        _SwerveDriveKinematics = new SwerveDriveKinematics(
            DriveConstants.LFLocation,
            DriveConstants.RFLocation,
            DriveConstants.LBLocation,
            DriveConstants.RBLocation
        );

        field = new Field2d();
        robotPose = new Pose2d();

        SwerveModulePosition[] swerveInitialPositions = {
            _LF.getPosition(), _RF.getPosition(), _LB.getPosition(), _RB.getPosition()
        };

        _SwerveDriveOdometry = new SwerveDriveOdometry(_SwerveDriveKinematics, getGyroHeading(), swerveInitialPositions, robotPose);
        _SwerveModules = new SwerveModule[]{_LF, _RF, _LB, _RB};
        _SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(_SwerveDriveKinematics, getGyroHeading(), swerveInitialPositions, robotPose);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
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

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.Autonomous || DebugSetting.TraceLevel == DebugLevel.All) {
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
        NetworkTableInstance.getDefault().getEntry("/Gyro").setDouble(Math.abs(_gyroscope.getYaw().getValueAsDouble()) % 360.0);
        SmartDashboard.putData(field);
        SmartDashboard.putData("Auto Pid x", xController);
        SmartDashboard.putData("Auto Pid y", yController);
        SmartDashboard.putData("Auto Pid z", headingController);
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

    public InstantCommand brakeCmd() {
        return new InstantCommand(() -> {this.brake();});
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
        _gyroscope.setYaw(180);
    }

    public void poseZero() {
        robotPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        _SwerveDriveOdometry.resetPose(robotPose);
    }

    public TeleopSwerve.JoystickAxes setJoystickAxes(TeleopSwerve.JoystickAxes axes) {
        joystickAxes = axes;
        return joystickAxes;
    }
    public void driveFieldRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = _SwerveDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                getGyroHeading()
            )
        );
    
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxRobotAutoSpeedmps);
    
        _LF.setDesiredState(states[0]);
        _RF.setDesiredState(states[1]);
        _LB.setDesiredState(states[2]);
        _RB.setDesiredState(states[3]);
    }
    public Pose2d getPose()
    {
        return robotPose;
    }

    public void resetOdometry(Pose2d pose) {
        // robotPose = pose;
        _SwerveDriveOdometry.resetPose(pose);
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = robotPose;

        double xC = xController.calculate(pose.getX(), sample.x);
        double yC = yController.calculate(pose.getY(), sample.y);
        double tC = headingController.calculate(pose.getRotation().getRadians(), sample.heading);

        if (DebugSetting.TraceLevel == DebugLevel.Autonomous || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("traj pid x", xC);
            SmartDashboard.putNumber("traj pid y", yC);
            SmartDashboard.putNumber("traj pid z", tC);
        }

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xC,
            sample.vy + yC,
            sample.omega + tC
        );

        // Apply the generated speeds
        driveFieldRelative(speeds);
    }
    private final SwerveSample[] emptyTrajectory = new SwerveSample[0];
    public SwerveSample[] currentTrajectory = emptyTrajectory;
  
      public void logTrajectory(Trajectory<SwerveSample> traj, boolean isStarting) {
        currentTrajectory = isStarting ? traj.samples().toArray(SwerveSample[]::new) : emptyTrajectory;
      }
        
}