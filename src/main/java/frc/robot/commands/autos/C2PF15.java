package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ArmRotateToSetpointCommand;
import frc.robot.commands.ElevatorToSetpointCommand;
import frc.robot.subsystems.ArmSubSys;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class C2PF15 extends SequentialCommandGroup {
    public C2PF15(
        AutoFactory autoFactory,
        DriveTrainSubsystem driveTrainSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubSys armSubSystem,
        ClawSubsystem clawSubsystem
    ) {
        addCommands(
            Commands.sequence(
                Commands.parallel(
                    autoFactory.resetOdometry("C 2P F1,5 1-3"), // Reset odometry & run the robot
                    new ElevatorToSetpointCommand(elevatorSubsystem, ElevatorAndArmConstants.ElevatorL4Posn) // Set the elevator to setpoint while moving
                ),
                new ArmRotateToSetpointCommand(armSubSystem, ElevatorAndArmConstants.ArmRotateL4Posn), // Move the arm while stopped
                Commands.deadline(new WaitCommand(0.5), new RunCommand(() -> {clawSubsystem.ClawRun(0.3);}, clawSubsystem)) // Run the claw for 0.5s
                // Next is the rest of the path.
            )
        );
    }
}
