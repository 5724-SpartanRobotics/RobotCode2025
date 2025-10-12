package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagLockonCommand extends Command {
    private final DriveTrainSubsystem _DriveTrainSubsystem;
    private final VisionSubsystem _VisionSubsystem;
    private final AprilTagFieldLayout _AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    private final PIDController _translationPid;
    private final PIDController _rotationPid;

    public AprilTagLockonCommand (
        DriveTrainSubsystem driveTrainSubsystem,
        VisionSubsystem visionSubsystem
    ) {
        addRequirements(driveTrainSubsystem, visionSubsystem);
                this._DriveTrainSubsystem = driveTrainSubsystem;
                this._VisionSubsystem = visionSubsystem;
        
                this._translationPid = new PIDController(1., 0., 0.1);
                this._rotationPid = new PIDController(2., 0., 0.2);
            }
        
            private void addRequirements(DriveTrainSubsystem driveTrainSubsystem, VisionSubsystem visionSubsystem) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'addRequirements'");
            }
        
            @Override
    public void execute() {
        // (hopefully) Average out the estimated pose and the MegaTag2 pose (which is fed into the pose estimator in VisionSubsystem)
        Pose2d robotPose = _DriveTrainSubsystem.getPoseEstimator().getEstimatedPosition()   
            .plus(new Transform2d(_VisionSubsystem.robotPose.getTranslation(), _VisionSubsystem.robotPose.getRotation()))
            .div(2);
        Pose2d targetPose = _AprilTagFieldLayout.getTagPose(_VisionSubsystem.currentFiducial).orElse(new Pose3d(robotPose)).toPose2d();
        Translation2d toTag = targetPose.getTranslation().minus(robotPose.getTranslation());
        double distanceToTag = toTag.getNorm();

        double angleToTag = Math.atan2(toTag.getY(), toTag.getX());
        double currentHdg = robotPose.getRotation().getRadians();
        double angleError = angleToTag - currentHdg;

        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

        double driveSpeed = _translationPid.calculate(distanceToTag, 0);
        double turnSpeed = _rotationPid.calculate(angleError, 0);

        _DriveTrainSubsystem.drive(new Translation2d(driveSpeed, 0), turnSpeed);
    }
}
