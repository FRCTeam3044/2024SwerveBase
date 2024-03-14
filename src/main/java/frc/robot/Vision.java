package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision {
    private double lastEstTimestamp = 0;
    private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Simulation
    private ArrayList<PhotonCameraSim> m_cameraSims = new ArrayList<PhotonCameraSim>();
    private VisionSystemSim visionSim;
    // Real
    private ArrayList<PhotonCamera> m_cameras = new ArrayList<PhotonCamera>();
    private ArrayList<PhotonPoseEstimator> m_poseEstimators = new ArrayList<PhotonPoseEstimator>();

    public Vision() {
        // for each camera in activeCameras, create a PhotonCamera and add it to
        // m_cameras
        for (int i = 0; i < Constants.VisionConstants.activeCameras.length; i++) {
            m_cameras.add(new PhotonCamera(Constants.VisionConstants.activeCameras[i]));
            // create a PhotonPoseEstimator and add it to m_poseEstimators for each camera
            m_poseEstimators.add(
                    new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cameras.get(i),
                            Constants.VisionConstants.cameraTransforms[i]));
            // set the fallback strategy for each PhotonPoseEstimator
            m_poseEstimators.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        // simulation
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(fieldLayout);
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(0));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(25);
            cameraProp.setAvgLatencyMs(20);
            cameraProp.setLatencyStdDevMs(15);

            for (int i = 0; i < Constants.VisionConstants.activeCameras.length; i++) {
                m_cameraSims.add(new PhotonCameraSim(m_cameras.get(i), cameraProp));
                visionSim.addCamera(m_cameraSims.get(i), Constants.VisionConstants.cameraTransforms[i]);
                m_cameraSims.get(i).enableDrawWireframe(true);
            }
        }
    }

    public PhotonPipelineResult getLatestResult(int cameraIndex) {
        return m_cameras.get(cameraIndex).getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per camera per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int cameraIndex) {
        m_poseEstimators.get(cameraIndex).setReferencePose(RobotContainer.m_robotDrive.getPose());
        var visionEst = m_poseEstimators.get(cameraIndex).update();
        double latestTimestamp = m_cameras.get(cameraIndex).getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est -> getSimDebugField()
                            .getObject("VisionEstimation")
                            .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult)
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
        if (newResult)
            lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, int cameraIndex) {
        var estStdDevs = Constants.VisionConstants.SingleTagStdDevs;
        var targets = getLatestResult(cameraIndex).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = m_poseEstimators.get(cameraIndex).getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // // Decrease std devs if multiple targets are visible
        // if (numTags > 1)
        // estStdDevs = Constants.VisionConstants.MultiTagStdDevs;
        // Increase std devs based on (average) distance
        // if (avgDist > 4)
        // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
        // Double.MAX_VALUE);
        // else
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 6));

        return estStdDevs;
    }

    // Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }
}