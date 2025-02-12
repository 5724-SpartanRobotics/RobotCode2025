package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constant {
   
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;
    public static final class CanIdConstants
    {
        public static final int LFTurnMotor = 2;
        public static final int LFDriveMotor = 3;
        public static final int LBTurnMotor = 4;
        public static final int LBDriveMotor = 5;
        public static final int RBTurnMotor = 6;
        public static final int RBDriveMotor = 7;
        public static final int RFTurnMotor = 8;
        public static final int RFDriveMotor = 9;
        public static final int LFCanID = 10;
        public static final int LBCanID = 11;
        public static final int RFCanID = 12;
        public static final int RBCanID = 13;
        public static final int PigeonID = 14;
        public static final int PDHID = 1;
        public static final int pneumaticontroller = 15;
        
     }
    public final class DriveConstants {
        public static final double maxRobotSpeedmps = 4.5;
        public static final double driveGearRatio = 6.75;
        public static double turnGearRatio = 150.0 / 7.0;
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
    
        // Maximum angular velocity (in radians per second)
        public static final double maxAngularVelocityRadps = Math.PI * 2;
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
        public static double joystickZDeadband = 0.3;//a deadband that you must overcome for the joystick input, otherwise we send 0
    }

    public final class ElevatorAndArmConstants
    {
        public static final int ArmRotateMtrCtrl1CanId = 18;
        public static final int ArmRotateMtrCtrl2CanId = 19;
        public static final int ArmExtendMtrCtrlCanId = 16;
        public static final int ElevatorMtrCtrl1CanId = 20;
        public static final int ElevatorMtrCtrl2CanId = 21;

        public static final double ArmExtendPidP = 0.0001;
        public static final double ArmExtendPidFF = 0.0002;
        public static final double ArmExtendPidI = 0;
        public static final double ArmExtendPidD = 0;

        public static final double ArmRotatePidP = 0.0001;
        public static final double ArmRotatePidFF = 0.0002;
        public static final double ArmRotatePidI = 0;
        public static final double ArmRotatePidD = 0;

        public static final double ElePidP = 0.0001;
        public static final double ElePidFF = 0.0002;
        public static final double ElePidI = 0;
        public static final double ElePidD = 0;
    }

    public static final class LedConstants {
        public static final int port = 0;
        public static final int stripLength = 60;
    }


    public static final class DebugSetting{
        //set this to get more values to smart dashboard. Leave it off for competition.
        public static final DebugLevel TraceLevel = DebugLevel.Swerve;
    }
    public static enum DebugLevel{
        Off,
        Swerve,
        Amp,
        Climber,
        IntakeShooter,
        Helix,
        Vision,
        All
    }
}
