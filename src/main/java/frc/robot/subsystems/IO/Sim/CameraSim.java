package frc.robot.subsystems.IO.Sim;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.CameraIO;

public class CameraSim implements CameraIO {
    /* MAIN CAMERA */
    private final PhotonCamera Main_Camera;
    private final PhotonPoseEstimator Main_PhotonEstimator;

    private final PhotonCameraSim Main_CameraSim;
    private final VisionSystemSim Main_VisionSim;

    public static final Matrix<N3, N1> Main_SingleTagStdDevs = Constants.Limelight.CameraProperties.singleTagStdDevs;
    public static final Matrix<N3, N1> Main_MultiTagStdDevs = Constants.Limelight.CameraProperties.multiTagStdDevs;

    private boolean Main_DoRejectUpdate;
    private Pose2d Main_EstimatedPose;
    private Matrix<N3, N1> Main_StdDevs;

    private PhotonTrackedTarget Main_CurrentTarget = null;

    /* BACK CAMERA */
    private final PhotonCamera Back_Camera;
    private final PhotonPoseEstimator Back_PhotonEstimator;

    private final PhotonCameraSim Back_CameraSim;
    private final VisionSystemSim Back_VisionSim;

    public static final Matrix<N3, N1> Back_SingleTagStdDevs = Constants.Limelight.CameraProperties.singleTagStdDevs;
    public static final Matrix<N3, N1> Back_MultiTagStdDevs = Constants.Limelight.CameraProperties.multiTagStdDevs;

    private boolean Back_DoRejectUpdate;
    private Pose2d Back_EstimatedPose;
    private Matrix<N3, N1> Back_StdDevs;

    private PhotonTrackedTarget Back_CurrentTarget = null;

    public CameraSim() {
        /* MAIN CAMERA */
        Main_Camera = new PhotonCamera("camera");
        Main_PhotonEstimator = new PhotonPoseEstimator(
            Constants.FIELD_LAYOUT, 
            Constants.Limelight.Main_RobotToCamera);
        Main_VisionSim = new VisionSystemSim("camera");
        Main_VisionSim.addAprilTags(Constants.FIELD_LAYOUT);

        // Limelight 3G
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(Constants.Limelight.CameraProperties.resWidth, Constants.Limelight.CameraProperties.resHeight, Constants.Limelight.CameraProperties.fovDiag);
        cameraProp.setCalibError(Constants.Limelight.CameraProperties.avgErrorPx, Constants.Limelight.CameraProperties.errorStdDevPx);
        cameraProp.setFPS(Constants.Limelight.CameraProperties.fps);
        cameraProp.setAvgLatencyMs(Constants.Limelight.CameraProperties.avgLatencyMs);
        cameraProp.setLatencyStdDevMs(Constants.Limelight.CameraProperties.latencyStdDevMs);

        Main_CameraSim = new PhotonCameraSim(Main_Camera, cameraProp);
        Main_VisionSim.addCamera(Main_CameraSim, Constants.Limelight.Main_RobotToCamera);

        // -------------------------
        Main_DoRejectUpdate = true;
        Main_EstimatedPose = new Pose2d();
        Main_StdDevs = Back_SingleTagStdDevs;

        /* BACK CAMERA */
        Back_Camera = new PhotonCamera("camera_back");
        Back_PhotonEstimator = new PhotonPoseEstimator(
            Constants.FIELD_LAYOUT, 
            Constants.Limelight.Back_RobotToCamera);
        Back_VisionSim = new VisionSystemSim("camera_back");
        Back_VisionSim.addAprilTags(Constants.FIELD_LAYOUT);

        // Limelight 3G
        var backCameraProp = new SimCameraProperties();
        backCameraProp.setCalibration(Constants.Limelight.CameraProperties.resWidth, Constants.Limelight.CameraProperties.resHeight, Constants.Limelight.CameraProperties.fovDiag);
        backCameraProp.setCalibError(Constants.Limelight.CameraProperties.avgErrorPx, Constants.Limelight.CameraProperties.errorStdDevPx);
        backCameraProp.setFPS(Constants.Limelight.CameraProperties.fps);
        backCameraProp.setAvgLatencyMs(Constants.Limelight.CameraProperties.avgLatencyMs);
        backCameraProp.setLatencyStdDevMs(Constants.Limelight.CameraProperties.latencyStdDevMs);

        Back_CameraSim = new PhotonCameraSim(Back_Camera, cameraProp);
        Back_VisionSim.addCamera(Back_CameraSim, Constants.Limelight.Back_RobotToCamera);

        // -------------------------
        Back_DoRejectUpdate = true;
        Back_EstimatedPose = new Pose2d();
        Back_StdDevs = Back_SingleTagStdDevs;
    }

    public void init() {
        // ...
    }

    public void updateInputs(CameraIOInputs inputs) {
        inputs.Main_DoRejectUpdate = Main_DoRejectUpdate;
        inputs.Main_EstimatedPose = Main_EstimatedPose;
        inputs.Main_TimestampSeconds = Timer.getFPGATimestamp();
        inputs.Main_StdDevs = Main_StdDevs.getData();

        inputs.Main_HasTarget = Main_CurrentTarget != null;
        inputs.Main_MainAprilTagID = Main_CurrentTarget != null ? Main_CurrentTarget.getFiducialId() : -1;
        inputs.Main_TA = Main_CurrentTarget != null ? Main_CurrentTarget.getArea() : 0;
        inputs.Main_MT1 = Main_EstimatedPose != null ? Main_EstimatedPose.getRotation().getDegrees() : 0;

        inputs.Back_DoRejectUpdate = Back_DoRejectUpdate;
        inputs.Back_EstimatedPose = Back_EstimatedPose;
        inputs.Back_TimestampSeconds = Timer.getFPGATimestamp();
        inputs.Back_StdDevs = Back_StdDevs.getData();

        inputs.Back_HasTarget = Back_CurrentTarget != null;
        inputs.Back_MainAprilTagID = Back_CurrentTarget != null ? Back_CurrentTarget.getFiducialId() : -1;
        inputs.Back_TA = Back_CurrentTarget != null ? Back_CurrentTarget.getArea() : 0;
        inputs.Back_MT1 = Back_EstimatedPose != null ? Back_EstimatedPose.getRotation().getDegrees() : 0;
    }

    public void periodic() {
        Main_Periodic();
        Back_Periodic();
    }

    public void Main_Periodic() {
        Main_VisionSim.update(SubsystemManager.getDrivebase().getSimulationPose());

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : Main_Camera.getAllUnreadResults()) {
            visionEst = Main_PhotonEstimator.estimateCoprocMultiTagPose(change);
            if (visionEst.isEmpty()) {
                visionEst = Main_PhotonEstimator.estimateLowestAmbiguityPose(change);
            }
            Main_UpdateStdDevs(visionEst, change.getTargets());

            Main_DoRejectUpdate = visionEst.isEmpty();
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                Main_DoRejectUpdate = true;

            visionEst.ifPresentOrElse(
                est ->
                    Main_VisionSim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                () -> {
                    Main_VisionSim.getDebugField().getObject("VisionEstimation").setPoses();
                });

            visionEst.ifPresent(est -> {
                Main_EstimatedPose = est.estimatedPose.toPose2d();
            });
        }
    }

    private void Main_UpdateStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            Main_StdDevs = Main_SingleTagStdDevs;
        } else {
            var estStdDevs = Main_SingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            double minDist = Double.MAX_VALUE;

            for (var tgt : targets) {
                var tagPose = Main_PhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;

                double dist = tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                if(dist < minDist) {
                    minDist = dist;
                    Main_CurrentTarget = tgt;
                }

                numTags++;
                avgDist += dist;
            }

            if (numTags == 0) {
                Main_StdDevs = Main_SingleTagStdDevs;
                Main_CurrentTarget = null;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = Main_MultiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                Main_StdDevs = estStdDevs;
            }
        }
    }

    public void Back_Periodic() {
        Back_VisionSim.update(SubsystemManager.getDrivebase().getSimulationPose());

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : Back_Camera.getAllUnreadResults()) {
            visionEst = Back_PhotonEstimator.estimateCoprocMultiTagPose(change);
            if (visionEst.isEmpty()) {
                visionEst = Back_PhotonEstimator.estimateLowestAmbiguityPose(change);
            }
            Back_UpdateStdDevs(visionEst, change.getTargets());

            Back_DoRejectUpdate = visionEst.isEmpty();
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                Back_DoRejectUpdate = true;

            visionEst.ifPresentOrElse(
                est ->
                    Back_VisionSim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                () -> {
                    Back_VisionSim.getDebugField().getObject("VisionEstimation").setPoses();
                });

            visionEst.ifPresent(est -> {
                Back_EstimatedPose = est.estimatedPose.toPose2d();
            });
        }
    }

    private void Back_UpdateStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            Back_StdDevs = Back_SingleTagStdDevs;
        } else {
            var estStdDevs = Back_SingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            double minDist = Double.MAX_VALUE;

            for (var tgt : targets) {
                var tagPose = Back_PhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;

                double dist = tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                if(dist < minDist) {
                    minDist = dist;
                    Back_CurrentTarget = tgt;
                }

                numTags++;
                avgDist += dist;
            }

            if (numTags == 0) {
                Back_StdDevs = Back_SingleTagStdDevs;
                Back_CurrentTarget = null;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = Back_MultiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                Back_StdDevs = estStdDevs;
            }
        }
    }

    @Override
    public void close() throws Exception {
        // ...
    }
}
