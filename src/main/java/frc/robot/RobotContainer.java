package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ElevatorAndArmConstants;
import frc.robot.commands.ArmRotateToSetpointCommand;
import frc.robot.commands.ElevatorToSetpointCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WristRotateToSetpointCommand;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubSys;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubSys;

public class RobotContainer {
    public final PowerDistribution _PowerDistribution = new PowerDistribution(CanIdConstants.PDHID, ModuleType.kRev);
    
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private TeleopSwerve _TeleopSwerve;
    private LedSubsystem _LedSubsystem;
    private ArmSubSys _ArmSubSys;
    private ElevatorSubsystem _ElevatorSubSys;
    private ClawSubsystem _ClawSubsystem;
    private AlgaeSubsystem _AlgaeSubSys;
    private WristSubSys _WristSubSys;
    private ClimberSubsystem _ClimberSubsystem;
    @SuppressWarnings("unused")
    private VisionSubsystem _VisionSubSys;
    private CommandJoystick _DriverController;
    private CommandJoystick _OperatorController;
    private Autos _Autos;

    public static Interference InterferenceHelper;
    public final SendableChooser<Command> m_autos = new SendableChooser<Command>();


    public RobotContainer(){
        _DriveTrainSubsystem = new DriveTrainSubsystem();
        _LedSubsystem = new LedSubsystem();
        _VisionSubSys = new VisionSubsystem(_DriveTrainSubsystem);
        _ArmSubSys = new ArmSubSys();
        _ElevatorSubSys = new ElevatorSubsystem();
        _ClawSubsystem = new ClawSubsystem(_LedSubsystem);
        _AlgaeSubSys = new AlgaeSubsystem();
        _WristSubSys = new WristSubSys();
        _ClimberSubsystem = new ClimberSubsystem();
        InterferenceHelper = new Interference(_ArmSubSys);
        _DriverController = new CommandJoystick(0);
        _OperatorController = new CommandJoystick(1);      

        _TeleopSwerve = new TeleopSwerve(_DriveTrainSubsystem, _DriverController);
        _Autos = new Autos(_DriveTrainSubsystem, _ElevatorSubSys, _ArmSubSys, _ClawSubsystem, _WristSubSys);

        _DriveTrainSubsystem.setDefaultCommand(_TeleopSwerve);
        CameraServer.startAutomaticCapture(); // Start camera server on zeroth index video device
        CameraServer.startAutomaticCapture(); // Start camera server on first index video device (auto inc)

        _DriveTrainSubsystem.zeroGyro();
        _DriveTrainSubsystem.flipGyro();
       

        configureBindings();
        // autoChooser.addRoutine("Blueleaveleft", null);
        // autoChooser.addCmd("Basic 2s Leave", );
        // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        configureAutos();
    }

    private void configureAutos() {
        m_autos.setDefaultOption("!!! NO AUTO !!!", new Command() {});
        m_autos.addOption("B C 2P F1,5", _Autos.Blue_Center_2Piece_Faces15);
        m_autos.addOption("B L 2P F6,5", _Autos.Blue_Left_2Piece_Faces65);
        m_autos.addOption("B R 2P F2,3", _Autos.Blue_Right_2Piece_Faces23);
        m_autos.addOption("Basic 2s Leave", _Autos.Leave);

        SmartDashboard.putData("Auto choices", m_autos);
    }

    private void configureBindings()
    {
        _DriverController.button(7).onTrue(new InstantCommand(() -> {
            _DriveTrainSubsystem.zeroGyro();
        }));
        // _DriverController.button(11).whileTrue(new AprilTagLockonCommand(_DriveTrainSubsystem, _VisionSubSys));
       
        //algae controls
        _OperatorController.button(8).whileTrue(new RunCommand(() ->{
            _AlgaeSubSys.AlgaeRotateAtSpeed(0.3);
        }, _AlgaeSubSys));
        _OperatorController.button(8).onFalse(new InstantCommand(() -> {
            _AlgaeSubSys.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubSys));
        _OperatorController.button(10).whileTrue(new RunCommand(() ->{
            _AlgaeSubSys.AlgaeRotateAtSpeed(-0.3);
        }, _AlgaeSubSys));
        _OperatorController.button(10).onFalse(new InstantCommand(() -> {
            _AlgaeSubSys.AlgaeRotateAtSpeed(0);
        }, _AlgaeSubSys));
        _OperatorController.button(7).onTrue(new InstantCommand(() -> {
            _AlgaeSubSys.RotateToOut();
        }, _AlgaeSubSys));
        _OperatorController.button(11).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.RotateToIn();
        }, _AlgaeSubSys));
        _OperatorController.button(9).onTrue(new InstantCommand(() ->{
            _AlgaeSubSys.RotateToHoldAlgaePosn();
        }, _AlgaeSubSys));

        //elevator increment up and down
        _OperatorController.povUp().onTrue(new InstantCommand(() ->{
            _ElevatorSubSys.IncrementElevatorUp();
        }, _ElevatorSubSys));
        _OperatorController.povDown().onTrue(new InstantCommand(() ->{
            _ElevatorSubSys.DecrementElevatorUp();
        }, _ElevatorSubSys));

        //arm rotate - main Y axis
        _OperatorController.axisLessThan(1, -0.3).whileTrue(new RunCommand(() ->{
            _ArmSubSys.ArmRotateToPositionMoreThanCurrent();
        }, _ArmSubSys));
        _OperatorController.axisGreaterThan(1, 0.3).whileTrue(new RunCommand(() ->{
            _ArmSubSys.ArmRotateToPositionLessThanCurrent();
        }, _ArmSubSys));

        //wrist rotate
        _OperatorController.povRight().onTrue(new InstantCommand(() ->{
            _WristSubSys.WristToPosition(180);
        }, _WristSubSys));
        _OperatorController.povLeft().onTrue(new InstantCommand(() ->{
            _WristSubSys.WristToPosition(0);
        }, _WristSubSys));

        //claw run 
        _OperatorController.button(1).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0.3);
        }, _ClawSubsystem));
        _OperatorController.button(1).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0);
        }, _ClawSubsystem));
        _OperatorController.button(2).onFalse(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(0);
        }, _ClawSubsystem));
        _OperatorController.button(2).onTrue(new InstantCommand(() ->{
            _ClawSubsystem.ClawRun(-0.3);
        }, _ClawSubsystem));
        //sequencial commands
        SequentialCommandGroup l4CommandGroups = new SequentialCommandGroup(
            new ElevatorToSetpointCommand((_ElevatorSubSys), ElevatorAndArmConstants.ElevatorL4Posn)
            .alongWith(new WristRotateToSetpointCommand(_WristSubSys, 0))
            // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
            .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, ElevatorAndArmConstants.ArmRotateL4Posn)));
        SequentialCommandGroup l3CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorSubSys), ElevatorAndArmConstants.ElevatorL3Posn)
                .alongWith(new WristRotateToSetpointCommand(_WristSubSys, 0))
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, ElevatorAndArmConstants.ArmRotateL3Posn)));
        SequentialCommandGroup l2CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorSubSys), ElevatorAndArmConstants.ElevatorL2Posn)
                .alongWith(new WristRotateToSetpointCommand(_WristSubSys, 0))
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, ElevatorAndArmConstants.ArmRotateL2Posn)));
        SequentialCommandGroup l1CommandGroups = new SequentialCommandGroup(
                new ElevatorToSetpointCommand((_ElevatorSubSys), ElevatorAndArmConstants.ElevatorL1Posn)
                .alongWith(new WristRotateToSetpointCommand(_WristSubSys, 0))
                // .alongWith(new AlgaeRotateToSetpointCommand(_AlgaeSubSys, AlgaeConstants.AlgaeClearOfArmMinimumAngle))
                .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, ElevatorAndArmConstants.ArmRotateL1Posn)));
        SequentialCommandGroup returnHomeCommandGroup = new SequentialCommandGroup(
                new WristRotateToSetpointCommand(_WristSubSys, 0)
                    .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, 0.0))
                    .alongWith(new ElevatorToSetpointCommand(_ElevatorSubSys, 0.0)));
        SequentialCommandGroup coralPickupGroup = new SequentialCommandGroup(
            new ElevatorToSetpointCommand(_ElevatorSubSys, ElevatorAndArmConstants.ElevatorCoralPosn)
            .alongWith(new WristRotateToSetpointCommand(_WristSubSys, ElevatorAndArmConstants.WristMax))
            .alongWith(new ArmRotateToSetpointCommand(_ArmSubSys, ElevatorAndArmConstants.ArmRotateCoralPosn)));
        _OperatorController.button(5).onTrue(l4CommandGroups);
        _OperatorController.button(3).onTrue(l3CommandGroups);
        _OperatorController.button(6).onTrue(l2CommandGroups);
        _OperatorController.button(4).onTrue(coralPickupGroup);
        _OperatorController.button(12).onTrue(returnHomeCommandGroup);
    }

    public void robotFinishedBooting() {
        _LedSubsystem.setColorForDuration(LedSubsystem.kDefaultNotificationColor, 2);
    }

    /**
     * Calling stop on the PID controlled parts of a subsystem calls stop on their PID ramp
     * instances, which stops them from setting a reference to the SparkMax ClosedLoop position
     * control and makes their ramp outputs follow the current position feedback.
     */
    public void StopPidRamps()
    {
        _AlgaeSubSys.AlgaeStop();
        _ArmSubSys.ArmExtendStop();
        _ArmSubSys.ArmRotateStop();
        _ElevatorSubSys.ElevatorStop();
    }

    public void ElevtorToStartingHeight()
    {
        _ElevatorSubSys.ElevatorToPosition(1);
    }
}
