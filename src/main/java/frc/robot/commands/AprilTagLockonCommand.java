package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
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
}
