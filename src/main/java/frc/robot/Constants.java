package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
        public static final Current MaxCurrentLimit = Units.Amps.of(80);

        public static final double DefaultDriveScale = Units.Percent.of(30).in(Units.Value);

        public static final class Wheel {
            public static final Distance Radius = Units.Inches.of(2);
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

        public static final class OperatorMap {
            private static final double AxisThreshold = 0.3;

            public static final int AlgaeManualIncrement = 8;
            public static final int AlgaeManualDecrement = 10;
            public static final int AlgaeSetpointOut = 7;
            public static final int AlgaeSetpointIn = 11;
            public static final int AlgaeSetpointHold = 9;

            public static final String ElevatorIncrement = "POV Up";
            public static final String ElevatorDecrement = "POV Down";

            public static final int ArmRotateAxis = 1;
            public static final double ArmRotateIncrementThreshold = -AxisThreshold;
            public static final double ArmRotateDecrementThreshold = AxisThreshold;

            public static final String WristOut = "POV Right";
            public static final String WristIn = "POV Left";

            public static final int ClawExpel = 1;
            public static final int ClawIntake = 2;

            public static final int PresetHome = 12;
            public static final int PresetIntake = 4;
            public static final int PresetL4 = 5;
            public static final int PresetL3 = 3;
            public static final int PresetL2 = 6;
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
        public static final class Speeds {
            public static final double Stopped = 0.0;
            public static final double Increment = 0.3;
            public static final double Decrement = -0.3;
        }
        public static final Angle RotateMax = Units.Degrees.of(100.0);
        public static final Angle RotateMin = Units.Degrees.of(0.0);
        public static final Angle Increment = Units.Degrees.of(5.0);
        public static final Angle Decrement = Increment;
        public static final Angle RotateAccuracyThreshold = Units.Degrees.of(5.0);
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
        public static final Angle RotateAccuracyThreshold = Units.Degrees.of(10.0);
    }

    public static final class Claw {
        public static final Current CurrentLimit = Units.Amps.of(32.0);
        public static final Current HighCurrent = Units.Amps.of(15.0);

        public static final class Speeds {
            public static final double Stopped = 0.0;
            public static final double Intake = 0.6;
            public static final double Expel = 0.3;
        }
    }

    public static final class Elevator {
        public static final Measure<LinearVelocityUnit> RampRate = Units.InchesPerSecond.of(4.0);
        public static final PIDFfController PIDFf = new PIDFfController(0.0, 0.0, 0.0, 0.0);

        public static final double GearRatio = Units.Rotations.of(45.0).in(Units.Rotations);
        public static final Distance ChainSprocketDiameter = Units.Inches.of(4.0);

        public static final Current StallCurrent = Units.Amps.of(70.0);
        public static final Time StallTimeout = Units.Seconds.of(3.0);

        public static final class Positions {
            public static final Distance Home = Units.Inches.of(0.0);
            public static final Distance L1 = Units.Inches.of(1.0);
            public static final Distance L2 = Units.Inches.of(1.0);
            public static final Distance L3 = Units.Inches.of(6.0);
            public static final Distance L4 = Units.Inches.of(15.0);
            public static final Distance Intake = Units.Inches.of(42.0);
        }
        public static final Distance ExtendMax = Units.Inches.of(14.5);
        public static final Distance ExtendMin = Units.Inches.of(0.0);
        public static final Distance Increment = Units.Inches.of(1.0);
        public static final Distance Decrement = Increment;
        public static final Distance ExtendAccuracyThreshold = Units.Inches.of(1.0);
    }

    public static final class LED {
        public static final int Port = 0;
        public static final int StripLength = 200;
    }

    public static final class Wrist {
        public static final Measure<AngularVelocityUnit> RampRate = Units.DegreesPerSecond.of(80.0);

        public static final PIDFfController PIDFf = new PIDFfController(1.0, 2.25E-4, 0.0, 5.0E-3);
        public static final double IntegralMaxAccumulation = 0.2;

        public static final double GearRatio = Units.Rotations.of(25.0).in(Units.Rotations);

        public static final class Positions {
            public static final Angle In = RotateMin;
            public static final Angle Out = RotateMax;
        }
        public static final Angle RotateMax = Units.Degrees.of(180.0);
        public static final Angle RotateMin = Units.Degrees.of(0.0);
        public static final Angle Increment = Units.Degrees.of(1.0);
        public static final Angle Decrement = Increment;
        public static final Angle RotateAccuracyThreshold = Units.Degrees.of(10.0);
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
