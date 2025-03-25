package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public final class Constants {
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;
    public static final DebugLevel DebugTraceLevel = DebugLevel.All;

    public final class Robot {
        public static final edu.wpi.first.units.measure.Mass Mass = Units.Pounds.of(110.7);
        public static final edu.wpi.first.units.measure.MomentOfInertia MomentOfInteria = Units.KilogramSquareMeters.of(16.5);
        public static final double WheelLockTime = Units.Seconds.of(3.0).in(Units.Seconds);
    }

    public final class Drive implements WithMaxCurrentLimit {
        public static final edu.wpi.first.units.measure.LinearVelocity MaxRobotVelocity = Units.FeetPerSecond.of(14.5);
        public static final edu.wpi.first.units.measure.LinearAcceleration MaxRobotAcceletaion = Units.MetersPerSecondPerSecond.of(4.0);
        public static final int NumberOfMotors = 8;
        public static final edu.wpi.first.units.measure.Current MaxCurrentLimit = Units.Amps.of(80);

        public static final double DefaultDriveScale = Units.Percent.of(30).in(Units.Value);

        public final class Wheel {
            public static final edu.wpi.first.units.measure.Distance Radius = Units.Inches.of(2);
            public static final double COF = 1.2;

            public static final double Base = Units.Inches.of(20.75).in(Units.Meters);
            public static final double TrackWidth = Units.Inches.of(23.0).in(Units.Meters);
        }

        public final class SwerveModuleOffsets {
            public static final Translation2d LF = new Translation2d(Wheel.Base / 2, Wheel.TrackWidth / 2);
            public static final Translation2d RF = new Translation2d(Wheel.Base / 2, - Wheel.TrackWidth / 2);
            public static final Translation2d LB = new Translation2d(- Wheel.Base / 2, Wheel.TrackWidth / 2);
            public static final Translation2d RB = new Translation2d(- Wheel.Base / 2, - Wheel.TrackWidth / 2);
        }

        public final class YAGSL {
            public static final double AngularVelocityCompensationCoefficient = 0.1;
            public static final double ModuleEncoderAutoSynchronizeDeadband = 1;

            public final class HolonomicPIDs {
                public static final PIDConstants Translation = new PIDConstants(5D, 0D, 0D);
                public static final PIDConstants Rotation = new PIDConstants(5D, 0D, 0D);
            }
        }
    }

    public final class Controller {
        public static final double Deadband = 0.5;

        public final class DriverMap {
            public static final int ZeroGyro = 7;
            public static final int SysIdDrive = 6;
            public static final int DriveToPose = 9;
            public static final int Lock_TEST = 11;
            public static final int Lock = 3;
            public static final int AddFakeVisionReading = 11;
            public static final int DriveToDistance = 12;
            public static final int ZeroSwerveModules = 10;
        }
    }

    public static enum DebugLevel {
        Off,
        All,
        Autonomous,
        Swerve,
        ArmRotate,
        Elevator,
        Algae,
        Claw,
        Wrist,
        Vision;

        public static boolean is(DebugLevel level) {
            return DebugTraceLevel == level;
        }
        public static boolean isOrAll(DebugLevel level) {
            return is(level) || DebugTraceLevel == All;
        }
    }

    public interface WithMaxCurrentLimit {
        public static final edu.wpi.first.units.measure.Current MaxCurrentLimit = Units.Amps.of(Double.POSITIVE_INFINITY);
    }
}
