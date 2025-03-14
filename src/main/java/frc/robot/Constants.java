package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;

    public static final class CanIdConstants {
        public static final int LFTurnMotor = 7;
        public static final int LFDriveMotor = 8;
        public static final int LBTurnMotor = 6;
        public static final int LBDriveMotor = 5;
        public static final int RBTurnMotor = 3;
        public static final int RBDriveMotor = 4;
        public static final int RFTurnMotor = 2;
        public static final int RFDriveMotor = 1;
        public static final int LFCanID = 10;
        public static final int LBCanID = 11;
        public static final int RFCanID = 13;
        public static final int RBCanID = 12;
        public static final int PigeonID = 14;
        public static final int PDHID = 50;
        public static final int ArmRotateMtrCtrl1CanId = 19;
        public static final int ArmRotateMtrCtrl2CanId = 18;
        // public static final int ArmExtendMtrCtrlCanId = 16;
        public static final int ElevatorMtrCtrl1CanId = 20;
        public static final int ElevatorMtrCtrl2CanId = 21;
        public static final int AlgaeRotateMtrCtrlCanId = 22;
        public static final int ClawMtrCtrlCanId = 17;
        public static final int WristMtrCtrlCanId = 23;
        public static final int Climber = 24;
    }
    public final class DriveConstants {
        public static final double maxRobotSpeedmps = 4.5;
        public static final double driveGearRatio = 6.75;
        public static final double turnGearRatio = 150.0 / 7.0;
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
    
        // Maximum angular velocity (in radians per second)
        public static final double maxAngularVelocityRadps = TwoPI;
        public static final double maxLinearSpeed = 15.5F / 3.281F;
    
        // Wheelbase and track width (in meters)
        public static final double wheelBase = 0.6; // Distance between front and back wheels
        public static final double trackWidth = 0.6; // Distance between left and right wheels
    
        // Swerve module offsets (in radians) - adjust these based on calibration
        public static final double LFOff = 0.0;
        public static final double RFOff = 0.0;
        public static final double LBOff = 0.0;
        public static final double RBOff = 0.0; 
        public static Translation2d LFLocation = new Translation2d(wheelBase/2, trackWidth/2);
        public static Translation2d RFLocation = new Translation2d(wheelBase/2, -trackWidth/2);
        public static Translation2d LBLocation = new Translation2d(-wheelBase/2, trackWidth/2);
        public static Translation2d RBLocation = new Translation2d(-wheelBase/2, -trackWidth/2);
        public static final int kNominalVoltage = 12;
        public static final int kDriveCurrentLimit = 60;
        public static final int kSteerCurrentLimit = 25;
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
        public static double joystickZDeadband = 0.35;//a deadband that you must overcome for the joystick input, otherwise we send 0
    }

    public final class ElevatorAndArmConstants {
        public static final double ArmExtendPidP = 0.035;
        public static final double ArmExtendPidFF = 0.000325;
        public static final double ArmExtendPidI = 0;
        public static final double ArmExtendPidD = 0;

        public static final double ArmRotatePidP = 0.048;
        public static final double ArmRotatePidFF = 0.000;
        public static final double ArmRotatePidI = 0.00007;
        public static final double ArmRotatePidD = 0;

        public static final double ElePidP = 0.04;
        public static final double ElePidFF = 0.0025;
        public static final double ElePidI = 0;
        public static final double ElePidD = 0;

        public static final double ArmRotateGearRatio = 45.0;
        public static final double ArmExtendGearRatio = 12.0;
        public static final double ElevatorGearRatio = 12.0;
        public static final double ElevatorChainSproketDiameter = 1.5;//inches
        public static final double ArmExtendSpoolDiameter = 1.0;
        public static final double ArmPivotPointToFrameEdgeInches = 18.0;
        public static final double ArmLengthRetractedInches = 33.0;
        public static final double ArmMaxOutsideOfFrameInches = 17.0;
        public static final double ArmAlgaeZoneAngle = 5;//degrees
        public static final double ArmExtendMaxWhenInRobot = 1;//inches
        public static final double ElevatorL4Posn = 14.5;
        public static final double ElevatorL3Posn = 6;
        public static final double ElevatorL2Posn = 1;
        public static final double ElevatorL1Posn = 1;
        public static final double ElevatorCoralPosn = 13.5;
        public static final double ArmRotateL4Posn = 150;
        public static final double ArmRotateL3Posn = 122;
        public static final double ArmRotateL2Posn = 100;//20;
        public static final double ArmRotateL1Posn = 85;//20;
        public static final double ArmRotateMin = 24;
        public static final double ArmRotateCoralPosn = 42;//20;
        public static final double ArmExtendL4Posn = 10;
        public static final double ArmExtendL3Posn = 5;
        public static final double ArmExtendL2Posn = 5;
        public static final double ArmExtendL1Posn = 3;
        public static final double ElevatorSetpointRampRate = 4;//inches per second
        public static final double ArmRotateSetpointRampRate = 40;//degrees per second
        public static final double ArmExtendSetpointRampRate = 4;//inches per second

        public static final double ElevatorMax = 14.5; // inches
        public static final double ArmRotateMax = 150.0; // degrees
        public static final double ArmExtendMax = 12.0; // inches

        public static final double WristPidP = 1.0;
        public static final double WristPidFF = 0.005;
        public static final double WristPidI = 0.000225;
        public static final double WristPidD = 0;
        public static final double WristRotateMax = 180; //degrees
        public static final double WristSetpointRampRate = 80;//degrees per second
        public static final double WristGearRatio = 25.0;
        public static final double WristMax = 180;
       }

    public final class AlgaeConstants {
        public static final double RotatePidP = 0.5;
        public static final double RotatePidFF = 0.000;
        public static final double RotatePidI = 0.0000;
        public static final double RotatePidD = 0;
        public static final double GearRatio = 81;
        public static final double IncrementDegrees = 5;
        public static final double AlgaeMinAngleForArmRotationClearance = 35;//degrees
        public static final double AlgaeHoldGamepieceAngle = 30;
        public static final double AlgaeFullOutAngle = 105;
        public static final double AlgaeSetpointRampRate = 60;//degrees per second
        public static final double AlgaeClearOfArmMinimumAngle = 60;
        public static final double RotateMaxAccel = 3000; // RPM/s
        public static final double RotateMaxVelocity = 5000; // RPM
    }
    public static final class ClimberConstants {
        public static final double GearRatio = 108.0;
        public static final double RampRate = GearRatio / 2;
        public static final double ClimbAngle = 90;

        public static final double PidP = 0.0;
        public static final double PidI = 0.0;
        public static final double PidD = 0.0;
        public static final double PidFF = 0.0;
    }

    public static final class NeoConstants {
        public static final double CountsPerRevolution = 1.0;
    }
    public static final class LedConstants {
        public static final int port = 0;
        public static final int stripLength = 200;
    }


    public static final class DebugSetting{
        //set this to get more values to smart dashboard. Leave it off for competition.
        public static final DebugLevel TraceLevel = DebugLevel.All;
    }
    public static enum DebugLevel{
        Off,
        Swerve,
        ArmRotate,
        ArmExtend,
        Elevator,
        Algae,
        Claw,
        Autonomous,
        Vision,
        Wrist,
        Climber,
        All
    }
}
