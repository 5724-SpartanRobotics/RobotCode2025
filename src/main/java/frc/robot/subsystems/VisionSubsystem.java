package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static final String kLimelightName = "limelight";
    private final DriveTrainSubsystem _DriveTrainSubsystem;

    public Pose2d robotPose;
    public int currentFiducial = -1;

    public VisionSubsystem(DriveTrainSubsystem driveTrainSubsystem) {
        super();
        this._DriveTrainSubsystem = driveTrainSubsystem;
    }

    @Override
    public void periodic() {
        super.periodic();

        // This needs to be called every periodic cycle because it actually updates the estimated position.
        LimelightHelpers.SetRobotOrientation(
            kLimelightName, _DriveTrainSubsystem.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );
        // The docs say to only use wpiBlue because starting in 2024 all coordinates are based off of blue side only.
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        // Do not do anything if there is wild stuff happening
        if (
            Math.abs(_DriveTrainSubsystem.getGyroRate()) > 720 ||
            mt2 == null || (mt2 != null && mt2.tagCount == 0) ||
            (mt2 != null && mt2.pose == null)
        ) return;

        _DriveTrainSubsystem.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        _DriveTrainSubsystem.getPoseEstimator().addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        robotPose = mt2.pose;
        currentFiducial = java.util.Arrays.asList(mt2.rawFiducials).stream().findFirst().orElse(
            new LimelightHelpers.RawFiducial(-1, 0, 0, 0, 0, 0, -1.)
        ).id;
    }
}
