package frc.robot.subsystems;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class VisionSubsystem extends SubsystemBase {
    public static final AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public final Optional<VisionSystemSim> VisionSim;

    private Supplier<Pose2d> _CurrentPose;
    private Field2d _Field;

    public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field) {
        this._CurrentPose = currentPose;
        this._Field = field;

        if (Robot.isSimulation()) {
            VisionSim = Optional.of(new VisionSystemSim("Vision"));
            VisionSim.get().addAprilTags(FieldLayout);
            for (Cameras c : Cameras.values()) {
                c.addToVisionSim(VisionSim.get());
            }
            openSimCameraViews();
        } else VisionSim = Optional.empty();
    }

    public static Pose2d getAprilTagPose(int fiducial, Transform2d robotOffset) {
        Optional<Pose3d> atPose3d = FieldLayout.getTagPose(fiducial);
        if (atPose3d.isPresent()) return atPose3d.get().toPose2d().transformBy(robotOffset);
        else throw new RuntimeException("Cannot get AprilTag " + fiducial + " from field " + FieldLayout.toString() + " (does it exist?)");
    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent() && VisionSim.isPresent()) {
            VisionSim.get().update(swerveDrive.getSimulationDriveTrainPose().get());
        }

        for (Cameras c : Cameras.values()) {
            Optional<EstimatedRobotPose> poseEstimation = getEstimatedGlobalPose(c);
            if (poseEstimation.isPresent()) {
                EstimatedRobotPose pose = poseEstimation.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, c.curStdDevs);
            }
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> poseEstimation = camera.getEstimatedGlobalPose();
        if (Robot.isSimulation() && VisionSim.isPresent()) {
            Field2d debugField = VisionSim.get().getDebugField();
            poseEstimation.ifPresentOrElse(
                est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                () -> debugField.getObject("VisionEstimation").setPoses()
            );
        }
        return poseEstimation;
    }

    public double getDistanceFromAprilTag(int fiducial) {
        Optional<Pose3d> tag = FieldLayout.getTagPose(fiducial);
        return tag.map(p -> PhotonUtils.getDistanceToPose(_CurrentPose.get(), p.toPose2d())).orElse(-1D);
    }

    public PhotonTrackedTarget getTargetfromId(int fiducial, Cameras camera) {
        PhotonTrackedTarget target = null;
        for (PhotonPipelineResult res : camera.resultList) {
            if (res.hasTargets()) {
                for (PhotonTrackedTarget i : res.getTargets()) {
                    if (i.getFiducialId() == fiducial) return i;
                }
            }
        }
        return target;
    }

    public VisionSystemSim getVisionSim() {
        return VisionSim.get();
    }

    private void openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            // Open the camera stream in a browser window
        }
    }

    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for (Cameras c : Cameras.values()) {
            if (!c.resultList.isEmpty()) {
                PhotonPipelineResult latest = c.resultList.get(0);
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget t : targets) {
            if (FieldLayout.getTagPose(t.getFiducialId()).isPresent()) {
                Pose2d targetPose = FieldLayout.getTagPose(t.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }
        _Field.getObject("tracked targets").setPoses(poses);
    }

    enum Cameras {
        LEFT_CAM(
            "left",
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)),
            new Translation3d(
                Units.Inches.of(0).in(Units.Meters),
                Units.Inches.of(0).in(Units.Meters),
                Units.Inches.of(0).in(Units.Meters)
            ),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)
        );

        public final Alert latencyAlert;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        private final Matrix<N3, N1> singleTagStdDevs;
        private final Matrix<N3, N1> multiTagStdDevs;
        private final Transform3d robotToCamTransform;
        public Matrix<N3, N1> curStdDevs;
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        public PhotonCameraSim cameraSim;
        public List<PhotonPipelineResult> resultList = new ArrayList<>();

        Cameras(
            String name,
            Rotation3d robotToCamRotation,
            Translation3d robotTocamTranslation,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs
        ) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
            camera = new PhotonCamera(name);
            robotToCamTransform = new Transform3d(robotTocamTranslation, robotToCamRotation);
            poseEstimator = new PhotonPoseEstimator(FieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
                cameraProp.setCalibError(0.25, 0.08);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);
                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        public void addToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) systemSim.addCamera(cameraSim, robotToCamTransform);
        }

        public Optional<PhotonPipelineResult> getBestResult() {
            if (resultList.isEmpty()) return Optional.empty();
            PhotonPipelineResult bestResult = resultList.get(0);
            double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
            double currentAmbiguity = 0;
            for (PhotonPipelineResult r : resultList) {
                currentAmbiguity = r.getBestTarget().getPoseAmbiguity();
                if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
                    bestResult = r;
                    ambiguity = currentAmbiguity;
                }
            }
            return Optional.of(bestResult);
        }

        public Optional<PhotonPipelineResult> getLatestResult() {
            return resultList.isEmpty() ? Optional.empty() : Optional.of(resultList.get(0));
        }

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        public void updateUnreadResults() {
            double mostRecentTimestamp = resultList.isEmpty() ? 0.0 : resultList.get(0).getTimestampSeconds();
            // double currentTimestamp = Units.Microseconds.of(NetworkTablesJNI.now()).in(Units.Seconds);
            for (PhotonPipelineResult r : resultList) {
                mostRecentTimestamp = Math.max(mostRecentTimestamp, r.getTimestampSeconds());
            }

            resultList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
            // lastReadTimestamp = currentTimestamp;
            resultList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1);
            if (!resultList.isEmpty()) updateEstimatedGlobalPose();
        }

        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEstimation = Optional.empty();
            for (PhotonPipelineResult change : resultList) {
                visionEstimation = poseEstimator.update(change);
                updateEstimationStdDevs(visionEstimation, change.getTargets());
            }
            estimatedRobotPose = visionEstimation;
        }

        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) curStdDevs = singleTagStdDevs;
            else {
                Matrix<N3, N1> estStdDevs = singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                for (PhotonTrackedTarget t : targets) {
                    Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(t.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    numTags++;
                    avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) curStdDevs = singleTagStdDevs;
                else {
                    avgDist /= numTags;
                    if (numTags > 1) estStdDevs = multiTagStdDevs;
                    if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MIN_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    curStdDevs = estStdDevs;
                }
            }
        }
    }
}
