
package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PresetCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    private LedSubsystem _LedSubsystem;
    private ArmSubsystem _ArmSubsystem;
    private ElevatorSubsystem _ElevatorSubsystem;
    private ClawSubsystem _ClawSubsystem;
    private AlgaeSubsystem _AlgaeSubsystem;
    private WristSubsystem _WristSubsystem;
    // private ClimberSubsystem _ClimberSubsystem;
    @SuppressWarnings("unused")
    private VisionSubsystem _VisionSubsystem;
    private CommandJoystick _DriverController;
    private CommandJoystick _OperatorController;
    private Autos _Autos;

    // public final SendableChooser<Command> m_autos = new SendableChooser<Command>();
    public final AutoChooser m_autos = new AutoChooser();


    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _VisionSubsystem = new VisionSubsystem(_DriveTrainSubsystem);
        _ArmSubsystem = new ArmSubsystem();
        _ElevatorSubsystem = new ElevatorSubsystem();
        _ClawSubsystem = new ClawSubsystem(_LedSubsystem);
        _AlgaeSubsystem = new AlgaeSubsystem();
        _WristSubsystem = new WristSubsystem();
        // _ClimberSubsystem = new ClimberSubsystem();
        _DriverController = new CommandJoystick(0);
        _OperatorController = new CommandJoystick(1);      

        _TeleopSwerve = new TeleopSwerve(_DriveTrainSubsystem, _DriverController);
        PresetCommands presetCommands = new PresetCommands(_ElevatorSubsystem, _WristSubsystem, _ArmSubsystem);
        _Autos = new Autos(_DriveTrainSubsystem, _ElevatorSubsystem, _ArmSubsystem, _ClawSubsystem, _WristSubsystem, presetCommands);

        _DriveTrainSubsystem.setDefaultCommand(_TeleopSwerve);
        // CameraServer.startAutomaticCapture(); // L4 Camera // Start camera server on zeroth index video device
        // CameraServer.startAutomaticCapture(); // Algae Camera // Start camera server on first index video device (auto inc)
        // CameraServer.startAutomaticCapture(); // Barrel Camera // Start camera server on second index video device (auto inc)
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath()); // Expose the deploy/ dir for Elastic layout

        _DriveTrainSubsystem.zeroGyro();
        _DriveTrainSubsystem.flipGyro();
       

        configureSmartDashboardTopics();
        configureBindings(presetCommands);
        configureAutos();
    }

    private void configureAutos() {
        // m_autos.addCmd("B C 2P F1,5", _Autos::_Blue_Center_2Piece_Face15);
        // m_autos.addCmd("B L 2P F6,5", _Autos::_Blue_Left_2Piece_Faces65);
        // m_autos.addCmd("B R 2P F2,3", _Autos::_Blue_Right_2Piece_Faces23);
        m_autos.addCmd("Basic 2s Leave", _Autos::_Leave);
        m_autos.addCmd("Leave w/ Preload", _Autos::_LeaveWithWrist);
        m_autos.addCmd("1Pc from Center", _Autos::_OnePieceCenter);
        // m_autos.addCmd("10 ft", _Autos::_10ft);
        // m_autos.addCmd("claw run", _Autos::_ClawRun);

        SmartDashboard.putData("Auto choices", m_autos);
        RobotModeTriggers.autonomous().whileTrue(m_autos.selectedCommandScheduler());
    }

    private void configureBindings(PresetCommands presetCommands) {
        _DriverController.button(7).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.zeroGyro();
        }));
        _DriverController.button(8).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.poseZero();
        }));
        // _DriverController.button(8).onTrue(new InstantCommand(() ->{
        //     _ClimberSubsystem.SetToClimbPosition();
        // }));
        // _DriverController.button(10).onTrue(new InstantCommand(() ->{
        //     _ClimberSubsystem.SetToHomePosition();
        // }));
        // _DriverController.button(12).whileTrue(new RunCommand(() ->{
        //     _ClimberSubsystem.Climb(-0.3);
        // }));
        // _DriverController.button(12).onFalse(new InstantCommand(() ->{
        //     _ClimberSubsystem.Climb(0);
        // }));
        // _DriverController.button(11).whileTrue(new RunCommand(() ->{
        //     _ClimberSubsystem.Climb(-1.0);
        // }));
        // _DriverController.button(11).onFalse(new InstantCommand(() ->{
        //     _ClimberSubsystem.Climb(0);
        // }));
        // _DriverController.button(11).whileTrue(new AprilTagLockonCommand(_DriveTrainSubsystem, _VisionSubSys));
       
        //algae controls
        _OperatorController.button(8).whileTrue(new RunCommand(() ->{
            _AlgaeSubsystem.AlgaeRotateAtSpeed(0.3);
        }, _AlgaeSubsystem));
        _OperatorController.button(8).onFalse(new InstantCommand(() -> {
            _AlgaeSubsystem.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubsystem));
        _OperatorController.button(10).whileTrue(new RunCommand(() ->{
            _AlgaeSubsystem.AlgaeRotateAtSpeed(-0.3);
        }, _AlgaeSubsystem));
        _OperatorController.button(10).onFalse(new InstantCommand(() -> {
            _AlgaeSubsystem.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubsystem));
        _OperatorController.button(7).onTrue(new InstantCommand(() -> {
            _AlgaeSubsystem.RotateToOut();
        }, _AlgaeSubsystem));
        _OperatorController.button(11).onTrue(new InstantCommand(() ->{
            _AlgaeSubsystem.RotateToIn();
        }, _AlgaeSubsystem));
        _OperatorController.button(9).onTrue(new InstantCommand(() ->{
            _AlgaeSubsystem.RotateToHoldAlgaePosn();
        }, _AlgaeSubsystem));

        //elevator increment up and down
        _OperatorController.povUp().onTrue(new InstantCommand(() ->{
            _ElevatorSubsystem.IncrementElevatorUp();
        }, _ElevatorSubsystem));
        _OperatorController.povDown().onTrue(new InstantCommand(() ->{
            _ElevatorSubsystem.DecrementElevatorUp();
        }, _ElevatorSubsystem));

        //arm rotate - main Y axis
        _OperatorController.axisLessThan(1, -0.3).whileTrue(new RunCommand(() ->{
            _ArmSubsystem.ArmRotateToPositionMoreThanCurrent();
        }, _ArmSubsystem));
        _OperatorController.axisGreaterThan(1, 0.3).whileTrue(new RunCommand(() ->{
            _ArmSubsystem.ArmRotateToPositionLessThanCurrent();
        }, _ArmSubsystem));

        //wrist rotate
        _OperatorController.povRight().onTrue(new InstantCommand(() ->{
            _WristSubsystem.WristToPosition(180);
        }, _WristSubsystem));
        _OperatorController.povLeft().onTrue(new InstantCommand(() ->{
            _WristSubsystem.WristToPosition(0);
        }, _WristSubsystem));

        //claw run 
        _OperatorController.button(1).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(ClawConstants.ExpelSpeed);
        }, _ClawSubsystem));
        _OperatorController.button(1).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(ClawConstants.StoppedSpeed);
        }, _ClawSubsystem));
        _OperatorController.button(2).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(ClawConstants.StoppedSpeed);
            _LedSubsystem.reset();
        }, _ClawSubsystem));
        _OperatorController.button(2).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(ClawConstants.IntakeSpeed);
            _LedSubsystem.setColor(Color.kGoldenrod, true);
        }, _ClawSubsystem));

        //sequential commands
        _OperatorController.button(5).onTrue(presetCommands.L4);
        _OperatorController.button(3).onTrue(presetCommands.L3);
        _OperatorController.button(6).onTrue(presetCommands.L2);
        _OperatorController.button(4).onTrue(presetCommands.CoralPickup);
        _OperatorController.button(12).onTrue(presetCommands.ReturnHome);
    }

    private void configureSmartDashboardTopics() {
        SmartDashboard.putNumber("Joystick X Deadband", ControllerConstants.joystickXDeadband);
        SmartDashboard.putNumber("Joystick Y Deadband", ControllerConstants.joystickYDeadband);
        SmartDashboard.putNumber("Joystick Z Deadband", ControllerConstants.joystickZDeadband);
    }

    public void robotFinishedBooting() {
        _LedSubsystem.setColorForDuration(LedSubsystem.kDefaultNotificationColor, 2);
        _DriveTrainSubsystem.zeroGyro();
    }

    /**
     * Calling stop on the PID controlled parts of a subsystem calls stop on their PID ramp
     * instances, which stops them from setting a reference to the SparkMax ClosedLoop position
     * control and makes their ramp outputs follow the current position feedback.
     */
    public void StopPidRamps()
    {
        _AlgaeSubsystem.AlgaeStop();
        _ArmSubsystem.ArmRotateStop();
        _ElevatorSubsystem.ElevatorStop();
        _WristSubsystem.WristStop();
        // _ClimberSubsystem.ClimbStop();
    }

    public void ElevatorToStartingHeight()
    {
        _ElevatorSubsystem.ElevatorToPosition(1);
    }

    /**
     * By setting all PID references to 0 on disable, it <i>should</i> mean that they will not shoot
     * back to where they were when the robot was disabled.
     */
    public void ResetPidReferences() {
        _AlgaeSubsystem.resetReferences();
        _ArmSubsystem.resetReferences();
        _ElevatorSubsystem.resetReferences();
        _WristSubsystem.resetReferences();
        // _ClimberSubsystem.resetReferences();
    }
}
