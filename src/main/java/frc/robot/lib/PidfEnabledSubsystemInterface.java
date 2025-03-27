package frc.robot.lib;

public interface PidfEnabledSubsystemInterface {
    default void resetReferences() {}

    double calculateSetpoint(double value);
}
