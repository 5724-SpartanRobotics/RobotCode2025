package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DebugLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class PresetCommands {
    private static boolean _Initialized = false;
    private static volatile ElevatorSubsystem _ElevatorSubsystem;
    private static volatile ArmSubsystem _ArmSubsystem;
    private static volatile WristSubsystem _WristSubsystem;

    private PresetCommands() {}

    public static void initialize(
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem,
        WristSubsystem wristSubsystem
    ) {
        if (_Initialized && DebugLevel.is(DebugLevel.Off)) return;
        else if (_Initialized) throw new IllegalStateException("Preset commands are alreay initialized");

        if (elevatorSubsystem == null) throw new IllegalArgumentException("Elevator subsystem cannnot be null!");
        if (armSubsystem == null) throw new IllegalArgumentException("Arm subsystem cannnot be null!");
        if (wristSubsystem == null) throw new IllegalArgumentException("Wrist subsystem cannnot be null!");
        _ElevatorSubsystem = elevatorSubsystem;
        _ArmSubsystem = armSubsystem;
        _WristSubsystem = wristSubsystem;
        _Initialized = true;
    }

    private static void checkInitialization() {
        if (!_Initialized) throw new IllegalStateException("Preset commands have not been initialized!");
        if (_ElevatorSubsystem == null) throw new IllegalArgumentException("Elevator subsystem cannnot be null!");
        if (_ArmSubsystem == null) throw new IllegalArgumentException("Arm subsystem cannnot be null!");
        if (_WristSubsystem == null) throw new IllegalArgumentException("Wrist subsystem cannnot be null!");
    }

    public static Command Home() {
        checkInitialization();
        return Commands.parallel(
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.In),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.Home),
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.Home)
        );
    }

    public static Command Intake() {
        checkInitialization();
        return Commands.parallel(
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.Home),
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.Out),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.Intake)
        );
    }

    public static Command L1() {
        checkInitialization();
        return Commands.parallel(
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.L1),
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.In),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.L1)
        );
    }

    public static Command L2() {
        checkInitialization();
        return Commands.parallel(
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.L2),
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.In),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.L2)
        );
    }

    public static Command L3() {
        checkInitialization();
        return Commands.parallel(
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.L3),
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.In),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.L3)
        );
    }

    public static Command L4() {
        checkInitialization();
        return Commands.parallel(
            _ElevatorSubsystem.toSetpoint(ElevatorSubsystem.Position.L4),
            _WristSubsystem.toSetpoint(WristSubsystem.RotatePosition.In),
            _ArmSubsystem.toSetpoint(ArmSubsystem.RotatePosition.L4)
        );
    }
}
