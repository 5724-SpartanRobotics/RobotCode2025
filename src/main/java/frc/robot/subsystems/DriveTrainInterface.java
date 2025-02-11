package frc.robot.Subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public interface DriveTrainInterface {
    Rotation2d getGyroHeading();
    void drive(Translation2d translation, double rotation);
}