package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Constants.DebugSetting;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TeleopSwerve extends Command {
    private CommandJoystick controller;
    private DriveTrainSubsystem swerveDrive;

    public static final class JoystickAxes {
        public double axisX;
        public double axisY;
        public double axisZ;

        public JoystickAxes(double x, double y, double z) {
            this.axisX = x;
            this.axisY = y;
            this.axisZ = z;
        }

        public JoystickAxes() { this.axisX = 0; this.axisY = 0; this.axisZ = 0; }

        public JoystickAxes set(double x, double y, double z) { this.axisX = x; this.axisY = y; this.axisZ = z; return this; }
        public JoystickAxes setX(double x) {axisX = x; return this;}
        public JoystickAxes setY(double y) {axisY = y; return this;}
        public JoystickAxes setZ(double z) {axisZ = z; return this;}

        public double getX() { return axisX; }
        public double getY() { return axisY; }
        public double getZ() { return axisZ; }
    }

    private JoystickAxes joystickAxes = new JoystickAxes();

    /**
     * Drive Controller
     * @param swerveDrive The drive train subsystem
     * @param controller A joystick
     */
    public TeleopSwerve(DriveTrainSubsystem swerveDrive, CommandJoystick controller){
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        addRequirements((SubsystemBase)swerveDrive);
        if (TimedRobot.isSimulation() && DebugSetting.TraceLevel == DebugLevel.All)
        {
            //SmartDashboard.putNumber("JoyStickY", 0.0);
            //SmartDashboard.putNumber("JoyStickX", 0.0);
            //SmartDashboard.putNumber("JoyStickZ", 0.0);
        }
 }

    @Override
    public void execute(){
        if (DriverStation.isAutonomous()){
            return;
        } else if (controller.button(7).getAsBoolean()) {
            return;
        } else {
            double xAxis = -controller.getX();
            double yAxis = -controller.getY();
            double zAxis = -controller.getTwist();
        // This chunk of code locks certain joystick directions if buttons are pressed

            double speedMod = DriveConstants.defaultSpeedMod;
            if (controller.button(Constants.ControllerConstants.MediumSpeedButton).getAsBoolean() &&
                controller.button(Constants.ControllerConstants.FastSpeedButton).getAsBoolean()) speedMod = 1.2;
            else if (controller.button(Constants.ControllerConstants.MediumSpeedButton).getAsBoolean()) speedMod = 0.65;
            else if (controller.button(Constants.ControllerConstants.FastSpeedButton).getAsBoolean()) speedMod = 1.0;
            double speedModZ = speedMod * 0.9;

        // Power Array Auto Align Code
        // Conditional is a check for having a combination of buttons pressed


            double joystickXDeadband = SmartDashboard.getNumber("Joystick X Deadband", ControllerConstants.joystickXDeadband);
            double joystickYDeadband = SmartDashboard.getNumber("Joystick Y Deadband", ControllerConstants.joystickYDeadband);
            double joystickZDeadband = SmartDashboard.getNumber("Joystick Z Deadband", ControllerConstants.joystickZDeadband);
        
        
            xAxis = (Math.abs(xAxis) < joystickXDeadband) ? 0 : xAxis * speedMod;
            yAxis = (Math.abs(yAxis) < joystickYDeadband) ? 0 : yAxis * speedMod;
            zAxis = (Math.abs(zAxis) < joystickZDeadband) ? 0 : zAxis * speedModZ;

            double rotation = zAxis * DriveConstants.maxAngularVelocityRadps;
            if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
                SmartDashboard.putNumber("ControllerRotation", rotation);
                SmartDashboard.putNumber("ControllerX", xAxis);
                SmartDashboard.putNumber("ControllerY", yAxis);
            }
            Translation2d translation = new Translation2d(yAxis, xAxis).times(DriveConstants.maxRobotSpeedmps);
            joystickAxes.set(xAxis, yAxis, zAxis);
            swerveDrive.drive(translation, rotation);
            swerveDrive.setJoystickAxes(getJoystickAxes());
        }
    }

    /**
     * Get the computed and math-ed joystick axes.
     * @return Joystick axes: <samp>{x, y, z}</samp> 
     */
    public JoystickAxes getJoystickAxes() {
        return joystickAxes;
    }
}