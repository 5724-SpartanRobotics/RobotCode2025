package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class Autos {
    private final AutoFactory _AutoFactory;

    public final Leave Leave;
    public final BC2PF15 Blue_Center_2Piece_Faces15;
    public final BL2PF65 Blue_Left_2Piece_Faces65;
    public final BR2PF23 Blue_Right_2Piece_Faces23;

    public Autos(
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem
    ) {
        _AutoFactory = new AutoFactory(
            driveTrainSubsystem::getPose, // A function that returns the current robot pose
            driveTrainSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            driveTrainSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            driveTrainSubsystem
        );  

        Leave = new Leave(driveTrainSubsystem);
        Blue_Center_2Piece_Faces15 = new BC2PF15(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem);
        Blue_Left_2Piece_Faces65 = new BL2PF65(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem);
        Blue_Right_2Piece_Faces23 = new BR2PF23(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem);
    }
}
