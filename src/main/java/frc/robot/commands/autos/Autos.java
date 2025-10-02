package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ClawRunForDurationCommand;
import frc.robot.commands.PresetCommands;
import frc.robot.commands.SetpointCommands;
import frc.robot.commands.ClawRunForDurationCommand.ClawRunMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class Autos {
    private final AutoFactory _AutoFactory;

    public final Leave Leave;
    public final LeaveWithWrist LeaveWithWrist;
    public final BC2PF15 Blue_Center_2Piece_Faces15;
    public final BL2PF65 Blue_Left_2Piece_Faces65;
    public final BR2PF23 Blue_Right_2Piece_Faces23;
    public final SequentialCommandGroup TenFt;
    public final SequentialCommandGroup ClawRun;
    public final OnePieceCenterL1 OnePieceCenterL1;
    public final OnePieceCenterL3 OnePieceCenterL3;

    public Autos(
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem,
        PresetCommands presetCommands
    ) {
        _AutoFactory = new AutoFactory(
            driveTrainSubsystem::getPose, // A function that returns the current robot pose
            driveTrainSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            driveTrainSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled +
            driveTrainSubsystem
        );

        Leave = new Leave(driveTrainSubsystem);
        LeaveWithWrist = new LeaveWithWrist(driveTrainSubsystem, wristSubsystem);
        OnePieceCenterL1 = new OnePieceCenterL1(driveTrainSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, clawSubsystem, presetCommands);
        OnePieceCenterL3 = new OnePieceCenterL3(driveTrainSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, clawSubsystem, presetCommands);
        Blue_Center_2Piece_Faces15 = new BC2PF15(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem, presetCommands);
        Blue_Left_2Piece_Faces65 = new BL2PF65(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem);
        Blue_Right_2Piece_Faces23 = new BR2PF23(_AutoFactory, driveTrainSubsystem, elevatorSubsystem, armSubsystem, clawSubsystem, wristSubsystem);
        TenFt = new SequentialCommandGroup(
            _AutoFactory.resetOdometry("10ft"),
            _AutoFactory.trajectoryCmd("10ft")
        );
        ClawRun =  new SequentialCommandGroup(
            new SetpointCommands.ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorCoralPosn),
            new SequentialCommandGroup(
                new SetpointCommands.WristRotateToSetpointCommand(wristSubsystem, ElevatorAndArmConstants.WristMax)
                    .alongWith(new SetpointCommands.ArmRotateToSetpointCommand(armSubsystem, ElevatorAndArmConstants.ArmRotateCoralPosn))
            ),
            new ClawRunForDurationCommand(clawSubsystem, ClawRunMode.Intake, 5.0)
        );
    }

    public Command _OnePieceCenterL1() {
        return OnePieceCenterL1;
    }

    public Command _OnePieceCenterL3() {
        return OnePieceCenterL3;
    }

    public Command _Blue_Center_2Piece_Face15() {
        return Blue_Center_2Piece_Faces15;
    }

    public Command _Blue_Left_2Piece_Faces65() {
        return Blue_Left_2Piece_Faces65;
    }

    public Command _Blue_Right_2Piece_Faces23() {
        return Blue_Right_2Piece_Faces23;
    }

    public Command _Leave() {
        return Leave;
    }

    public Command _LeaveWithWrist() {
        return LeaveWithWrist;
    }

    public Command _10ft() {
        return TenFt;
    }

    public Command _ClawRun() {
        return ClawRun;
    }
}
