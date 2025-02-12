package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public interface DriveTrainInterface {
    Rotation2d getGyroHeading();
    DriveTrainSubsystem drive(Translation2d translation, double rotation);
}