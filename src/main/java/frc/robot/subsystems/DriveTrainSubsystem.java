package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DebugLevel;
import frc.robot.Robot;
import frc.robot.lib.XyPair;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveDrive _SwerveDrive;
    private final boolean _VisionDriveTest = false;
    private VisionSubsystem _Vision;

    private final PIDController xTrajController = new PIDController(10.0, 0, 0);
    private final PIDController yTrajController = new PIDController(10.0, 0, 0);
    private final PIDController zTrajController = new PIDController(7.5, 0, 0);

    private static final RobotConfig config = new RobotConfig(
        Constants.Robot.Mass, Constants.Robot.MomentOfInteria,
        new ModuleConfig(
            Constants.Drive.Wheel.Radius,
            Constants.Drive.MaxRobotVelocity,
            Constants.Drive.Wheel.COF,
            DCMotor.getNeoVortex(Constants.Drive.NumberOfMotors),
            Constants.Drive.MaxCurrentLimit,
            Constants.Drive.NumberOfMotors
        ), 
        Constants.Drive.SwerveModuleOffsets.LF,
        Constants.Drive.SwerveModuleOffsets.RF,
        Constants.Drive.SwerveModuleOffsets.LB,
        Constants.Drive.SwerveModuleOffsets.RB
    );

    public final AutoFactory AutoFactory;

    public DriveTrainSubsystem(String configFilename) {
        this(new File(Filesystem.getDeployDirectory(), configFilename));
    }

    public DriveTrainSubsystem(File directory) {
        boolean blueAlliance = Robot.isBlueAlliance();
        Pose2d startingPose = blueAlliance ?
            new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.kZero) :
            new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.k180deg) ;

        SwerveDriveTelemetry.verbosity = DebugLevel.isOrAll(DebugLevel.Swerve) ? TelemetryVerbosity.HIGH : TelemetryVerbosity.NONE;
        try { _SwerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Drive.MaxRobotVelocity.in(Units.MetersPerSecond), startingPose); }
        catch (IOException e) { throw new RuntimeException(e); }
        _SwerveDrive.setHeadingCorrection(false);
        _SwerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        _SwerveDrive.setAngularVelocityCompensation(true, true, Constants.Drive.YAGSL.AngularVelocityCompensationCoefficient);
        _SwerveDrive.setModuleEncoderAutoSynchronize(false, Constants.Drive.YAGSL.ModuleEncoderAutoSynchronizeDeadband);
        zTrajController.enableContinuousInput(-Math.PI, Math.PI);

        this.AutoFactory = new AutoFactory(
            this::getPose,
            this::resetOdometry,
            this::followTrajectory,
            true,
            this
        );

        if (_VisionDriveTest) {
            initPhotonVision();
            _SwerveDrive.stopOdometryThread();
        }
        initPathPlanner();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    public DriveTrainSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration ctrlCfg) {
        _SwerveDrive = new SwerveDrive(
            driveCfg, ctrlCfg, Constants.Drive.MaxRobotVelocity.in(Units.MetersPerSecond),
            new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.kZero)
        );
        this.AutoFactory = new AutoFactory(
            this::getPose,
            this::resetOdometry,
            this::followTrajectory,
            true,
            this
        );
    }

    @Override
    public void periodic() {
        if (_VisionDriveTest) {
            _SwerveDrive.updateOdometry();
            _Vision.updatePoseEstimation(_SwerveDrive);
        }
    }

    @Override
    public void simulationPeriodic() {}

    public void initPhotonVision() {
        _Vision = new VisionSubsystem(_SwerveDrive::getPose, _SwerveDrive.field);
    }

    public void initPathPlanner() {
        final boolean enableFeedForward = true;
        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            (speedsRobotRelative, moduleFeedForwards) -> {
                if (enableFeedForward) {
                    _SwerveDrive.drive(
                        speedsRobotRelative,
                        _SwerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces()
                    );
                } else {
                    _SwerveDrive.setChassisSpeeds(speedsRobotRelative);
                }
            },
            new PPHolonomicDriveController(Constants.Drive.YAGSL.HolonomicPIDs.Translation, Constants.Drive.YAGSL.HolonomicPIDs.Rotation),
            config,
            () -> !Robot.isBlueAlliance(),
            this
        );
        PathfindingCommand.warmupCommand().schedule();
    }

    public Command aimAtTarget(VisionSubsystem.Cameras camera) {
        return run(() -> {
            Optional<PhotonPipelineResult> resO = camera.getBestResult();
            if (resO.isPresent()) {
                PhotonPipelineResult res = resO.get();
                if (res.hasTargets()) {
                    drive(getTargetSpeeds(0, 0, Rotation2d.fromDegrees(res.getBestTarget().getYaw())));
                }
            }
        });
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            _SwerveDrive.getMaximumChassisVelocity(), Constants.Drive.MaxRobotAcceletaion.baseUnitMagnitude(),
            _SwerveDrive.getMaximumChassisAngularVelocity(), Units.Degrees.of(720).in(Units.Radians)
        );
        return AutoBuilder.pathfindToPose(pose, constraints, Units.MetersPerSecond.of(0));
    }

    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeeds) throws IOException, ParseException {
        SwerveSetpointGenerator sg = new SwerveSetpointGenerator(config, _SwerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSp = new AtomicReference<>(new SwerveSetpoint(
            _SwerveDrive.getRobotVelocity(), _SwerveDrive.getStates(), DriveFeedforwards.zeros(_SwerveDrive.getModules().length)
        ));
        AtomicReference<Double> previousTime = new AtomicReference<>();
        return startRun(
            () -> previousTime.set(Timer.getFPGATimestamp()),
            () -> {
                double newTime = Timer.getFPGATimestamp();
                SwerveSetpoint nSp = sg.generateSetpoint(prevSp.get(), robotRelativeChassisSpeeds.get(), newTime - previousTime.get());
                _SwerveDrive.drive(nSp.robotRelativeSpeeds(), nSp.moduleStates(), nSp.feedforwards().linearForces());
                prevSp.set(nSp);
                previousTime.set(newTime);
            }
        );
    }

    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try { return driveWithSetpointGenerator(() -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading())); }
        catch (Exception e) { DriverStation.reportError(e.toString(), true); }
        return Commands.none();
    }

    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, _SwerveDrive, 12, true),
            3D, 5D, 3D
        );
    }

    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(_SwerveDrive.getModules()).forEach(it -> it.setAngle(0D)));
    }

    public Command driveToDistanceCommand(Distance distance, Measure<LinearVelocityUnit> speed) {
        return run(() -> drive(new ChassisSpeeds(speed.in(MetersPerSecond), 0D, 0D)))
            .until(() -> _SwerveDrive.getPose().getTranslation().getDistance(new Translation2d()) > distance.in(Units.Meters));
    }

    public Command driveToDistanceCommand(double distance_m, double speed_mps) {
        return driveToDistanceCommand(Units.Meters.of(distance_m), Units.MetersPerSecond.of(speed_mps));
    }

    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        _SwerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> _SwerveDrive.drive(
            SwerveMath.scaleTranslation(
                new Translation2d(
                    translationX.getAsDouble() * _SwerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * _SwerveDrive.getMaximumChassisVelocity()
                ), 
                0.8
            ),
            Math.pow(angularRotationX.getAsDouble(), 3) * _SwerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false
        ));
    }

    public Command driveCommand(
        DoubleSupplier translationX, DoubleSupplier translationY,
        DoubleSupplier headingX, DoubleSupplier headingY
    ) {
        _SwerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            driveFieldOriented(_SwerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(), scaledInputs.getY(),
                headingX.getAsDouble(), headingY.getAsDouble(),
                _SwerveDrive.getOdometryHeading().getRadians(),
                _SwerveDrive.getMaximumChassisVelocity()
            ));
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        _SwerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public Command driveCmd(Translation2d translation, double rotation, boolean fieldRelative) {
        return run(() -> {drive(translation, rotation, fieldRelative);});
    }

    public void drive(ChassisSpeeds velocity) {
        _SwerveDrive.drive(velocity);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        _SwerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> _SwerveDrive.driveFieldOriented(velocity.get()));
    }

    public void resetOdometry(Pose2d initialPose) {
        _SwerveDrive.resetOdometry(initialPose);
    }

    public Pose2d getPose() {
        return _SwerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        _SwerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        _SwerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        _SwerveDrive.zeroGyro();
    }

    /**
     * <b>EXPERIMENTAL
     */
    public void flipGyro() {
        _SwerveDrive.getGyro().setInverted(true);
    }

    public Command flipGyroCmd() {
        return run(() -> flipGyro());
    }

    private boolean isRedAlliance() {
        return !Robot.isBlueAlliance();
    }

    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
        }
        else zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        _SwerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return _SwerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(), scaledInputs.getY(),
            headingX, headingY, getHeading().getRadians(),
            Constants.Drive.MaxRobotVelocity.in(Units.MetersPerSecond)
        );
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return _SwerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(), scaledInputs.getY(),
            angle.getRadians(), getHeading().getRadians(),
            Constants.Drive.MaxRobotVelocity.in(Units.MetersPerSecond)
        );
    }

    public ChassisSpeeds getFieldVelocity() {
        return _SwerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return _SwerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return _SwerveDrive.getSwerveController();
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return _SwerveDrive.swerveDriveConfiguration;
    }

    public void lock() {
        _SwerveDrive.lockPose();
    }

    public DriveTrainSubsystem brake() {
        for (SwerveModule m : _SwerveDrive.getModules()) {
            m.setDesiredState(
                new SwerveModuleState(0, m.getState().angle),
                false, true
            );
        }
        return this;
    }

    public InstantCommand brakeCmd() {
        return new InstantCommand(() -> {this.brake();});
    }

    public Rotation2d getPitch() {
        return _SwerveDrive.getPitch();
    }

    public void addFakeVisionReading() {
        _SwerveDrive.addVisionMeasurement(new Pose2d(3D, 3D, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    public SwerveDrive getSwerveDrive() {
        return _SwerveDrive;
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xTrajController.calculate(pose.getX(), sample.x),
            sample.vy + yTrajController.calculate(pose.getY(), sample.y),
            sample.omega + zTrajController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        drive(speeds);
    }

    public static SwerveInputStream driveAngularVelocity(
        SwerveDrive swerveDrive, Supplier<XyPair> translationSupplier, Supplier<XyPair> rotationSupplier
    ) {
        return SwerveInputStream.of(
            swerveDrive,
            () -> translationSupplier.get().getY() * -1,
            () -> translationSupplier.get().getX() * -1
        )
        .withControllerRotationAxis(() -> rotationSupplier.get().getX())
        .deadband(Constants.Controller.Deadband)
        .scaleTranslation(Constants.Drive.DefaultDriveScale)
        .allianceRelativeControl(true);
    }

    public static SwerveInputStream driveDirectAngle(
        SwerveDrive swerveDrive, Supplier<XyPair> translationSupplier, Supplier<XyPair> rotationSupplier
    ) {
        return driveAngularVelocity(swerveDrive, translationSupplier, rotationSupplier).copy()
            .withControllerHeadingAxis(rotationSupplier.get()::getX, rotationSupplier.get()::getY)
            .headingWhile(true);
    }

    public static SwerveInputStream driveRobotOriented(
        SwerveDrive swerveDrive, Supplier<XyPair> translationSupplier, Supplier<XyPair> rotationSupplier
    ) {
        return driveAngularVelocity(swerveDrive, translationSupplier, rotationSupplier)
            .robotRelative(true)
            .allianceRelativeControl(false);
    }

    public static SwerveInputStream driveAngularVelocityKbd(
        SwerveDrive swerveDrive, Supplier<XyPair> translationSupplier, DoubleSupplier rawAxisSupplier
    ) {
        return SwerveInputStream.of(
            swerveDrive,
            () -> -translationSupplier.get().getY(),
            () -> -translationSupplier.get().getX()
        )
        .withControllerRotationAxis(rawAxisSupplier)
        .deadband(Constants.Controller.Deadband)
        .scaleTranslation(Constants.Drive.DefaultDriveScale)
        .allianceRelativeControl(true);
    }

    public static SwerveInputStream driveDirectAngleKbd(
        SwerveDrive swerveDrive, Supplier<XyPair> translationSupplier, DoubleSupplier rawAxisSupplier
    ) {
        return driveAngularVelocityKbd(swerveDrive, translationSupplier, rawAxisSupplier).copy()
            .withControllerHeadingAxis(
                () -> Math.sin(rawAxisSupplier.getAsDouble() * Math.PI) * Constants.TwoPI,
                () -> Math.cos(rawAxisSupplier.getAsDouble() * Math.PI) * Constants.TwoPI
            )
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.kZero);
    }
}