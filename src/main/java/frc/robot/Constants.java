package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.lib.PIDFfController;

public final class Constants {
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;
    public static final DebugLevel DebugTraceLevel = DebugLevel.All;

    public static final class CanIds {
        public static final class Swerve {
            public static final int RFDriveMotor = 1;
            public static final int RFTurnMotor = 2;
    
            public static final int RBDriveMotor = 4;
            public static final int RBTurnMotor = 3;
    
            public static final int LBDriveMotor = 5;
            public static final int LBTurnMotor = 6;
    
            public static final int LFTurnMotor = 7;
            public static final int LFDriveMotor = 8;
    
            public static final int LFCanID = 10;
            public static final int LBCanID = 11;
            public static final int RFCanID = 13;
            public static final int RBCanID = 12;
        }

        public static final int Pigeon2 = 14;

        public static final int Claw = 17;

        public static final int ArmRotateMotor2 = 18;
        public static final int ArmRotateMotor1 = 19;

        public static final int ElevatorMotor1 = 20;
        public static final int ElevatorMotor2 = 21;

        public static final int Algae = 22;

        public static final int Wrist = 23;

        public static final int PDH = 50;
    }

    public static final class Robot {
        public static final edu.wpi.first.units.measure.Mass Mass = Units.Pounds.of(110.7);
        public static final edu.wpi.first.units.measure.MomentOfInertia MomentOfInteria = Units.KilogramSquareMeters.of(16.5);
        public static final double WheelLockTime = Units.Seconds.of(10.0).in(Units.Seconds);
    }

    public static final class Drive {
        public static final edu.wpi.first.units.measure.LinearVelocity MaxRobotVelocity = Units.FeetPerSecond.of(14.5);
        public static final edu.wpi.first.units.measure.LinearAcceleration MaxRobotAcceletaion = Units.MetersPerSecondPerSecond.of(4.0);
        public static final int NumberOfMotors = 8;
        public static final edu.wpi.first.units.measure.Current MaxCurrentLimit = Units.Amps.of(80);

        public static final double DefaultDriveScale = Units.Percent.of(30).in(Units.Value);

        public static final class Wheel {
            public static final edu.wpi.first.units.measure.Distance Radius = Units.Inches.of(2);
            public static final double COF = 1.2;

            public static final double Base = Units.Inches.of(20.75).in(Units.Meters);
            public static final double TrackWidth = Units.Inches.of(23.0).in(Units.Meters);
        }

        public static final class SwerveModuleOffsets {
            public static final Translation2d LF = new Translation2d(Wheel.Base / 2, Wheel.TrackWidth / 2);
            public static final Translation2d RF = new Translation2d(Wheel.Base / 2, - Wheel.TrackWidth / 2);
            public static final Translation2d LB = new Translation2d(- Wheel.Base / 2, Wheel.TrackWidth / 2);
            public static final Translation2d RB = new Translation2d(- Wheel.Base / 2, - Wheel.TrackWidth / 2);
        }

        public static final class YAGSL {
            public static final double AngularVelocityCompensationCoefficient = 0.1;
            public static final double ModuleEncoderAutoSynchronizeDeadband = 1;

            public static final class HolonomicPIDs {
                public static final PIDConstants Translation = new PIDConstants(5D, 0D, 0D);
                public static final PIDConstants Rotation = new PIDConstants(5D, 0D, 0D);
            }
        }
    }

    public static final class Controller {
        public static final double Deadband = 0.5;

        public static final class DriverMap {
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

    public static final class Motor {
        public static final class NeoV1 {
            public static final double CountsPerRevolution = 1.0;
        }
    }

    public static final class Algae {
        public static final Measure<AngularVelocityUnit> RampRate = Units.DegreesPerSecond.of(60.0);

        public static final PIDFfController PIDFf = new PIDFfController(0.5, 1.0E-5, 0.0, 0.0);
        public static final double IntegralMaxAccumulation = 0.1;

        public static final double GearRatio = Units.Rotations.of(81.0).in(Units.Rotations);

        public static final class Positions {
            public static final Angle Extend = Units.Degrees.of(105.0);
            public static final Angle Retract = Units.Degrees.of(0.0);
            public static final Angle Hold = Units.Degrees.of(30.0);
        }
        public static final Angle RotateMax = Units.Degrees.of(100.0);
        public static final Angle RotateMin = Units.Degrees.of(0.0);
        public static final Angle Increment = Units.Degrees.of(5.0);
        public static final Angle Decrement = Increment;
    }

    public static final class Arm {
        public static final Measure<AngularVelocityUnit> RampRate = Units.DegreesPerSecond.of(40.0);

        public static final PIDFfController PIDFf = new PIDFfController(.048, 7.0E-5, 0.0, 0.0);
        public static final double IntegralMaxAccumulation = 0.2;

        public static final double GearRatio = Units.Rotations.of(45.0).in(Units.Rotations);

        public static final class Positions {
            public static final Angle Home = Units.Degrees.of(0.0);
            public static final Angle L1 = Units.Degrees.of(90.0);
            public static final Angle L2 = Units.Degrees.of(110.0);
            public static final Angle L3 = Units.Degrees.of(123.0);
            public static final Angle L4 = Units.Degrees.of(146.0);
            public static final Angle Intake = Units.Degrees.of(42.0);
        }
        public static final Angle RotateMax = Units.Degrees.of(150.0);
        public static final Angle RotateMin = Units.Degrees.of(24.0);
        public static final Angle Increment = Units.Degrees.of(10.0);
        public static final Angle Decrement = Units.Degrees.of(15.0);
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
}
