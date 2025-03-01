package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Leave extends SequentialCommandGroup {
    public Leave(DriveTrainSubsystem driveTrainSubsystem) {
        addRequirements(driveTrainSubsystem);
        addCommands(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(3),
                    new RunCommand(() -> {
                        driveTrainSubsystem.drive(new Translation2d(0.0, 1.0).times(DriveConstants.maxRobotSpeedmps), 0);
                    })
                )
            )
        );
    }
}
