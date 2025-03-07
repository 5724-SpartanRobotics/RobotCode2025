package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.ArmSubSys;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public final class Autos {
    private final AutoFactory _AutoFactory;

    public final Leave Leave;
    public final C2PF15 Center_2Piece_Faces15;

    public Autos(
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubSys armSubsystem,
        ClawSubsystem clawSubsystem
    ) {
        _AutoFactory = new AutoFactory(
            driveTrainSubsystem::getPose, // A function that returns the current robot pose
            driveTrainSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            driveTrainSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            driveTrainSubsystem
        );  

        Leave = new Leave(driveTrainSubsystem);
        Center_2Piece_Faces15 = new C2PF15(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem);
    }
}
