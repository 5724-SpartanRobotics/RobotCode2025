package frc.robot.commands;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.OnePieceCenter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class Autos {
    public static final AutoChooser Chooser = new AutoChooser();

    private final OnePieceCenter _OnePieceCenter;

    public Autos(
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem,
        ClawSubsystem clawSubsystem,
        WristSubsystem wristSubsystem
    ) {
        _OnePieceCenter = new OnePieceCenter(driveTrainSubsystem, elevatorSubsystem, armSubsystem, wristSubsystem, clawSubsystem);
    }

    public Command OnePieceCenter() {
        return _OnePieceCenter;
    }
}
