package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import limelight.Limelight;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.AngularVelocity3d;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;


public class VisionSubsystem {

    private final Limelight limelight;
    private final LimelightPoseEstimator poseEstimator;
    private final Pigeon2 gyro;
    public int currentFiducial;
    public Object robotPose;

    /**
     * Constructor: pass the gyro and the pose estimator in — don't create them silently here.
     */
    public VisionSubsystem(Pigeon2 gyro, LimelightPoseEstimator poseEstimator) {
        this.gyro = gyro;
        this.poseEstimator = poseEstimator;
        this.limelight = new Limelight("limelight");

        this.limelight.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(new Pose3d()) 
            .save();

        Orientation3d orientation = new Orientation3d(
           gyro.getRotation3d(), null, null, null);
            new AngularVelocity3d(
                 DegreesPerSecond.of(gyro.getPitch())
                 DegreesPerSecond.of(gyro.getRoll())  
                 DegreesPerSecond.of(gyro.getYaw())   
            )
        );

        this.limelight.getSettings()
            .withRobotOrientation(orientation)
            .save();

        Optional<PoseEstimate> visionEstimate = this.poseEstimator.getPoseEstimate();

        visionEstimate.ifPresent(poseEstimate -> {
            this.poseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        });
    }
}
