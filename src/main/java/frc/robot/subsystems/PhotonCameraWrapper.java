package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper extends TimedRobot{
    private PhotonCamera photonCamera1;
    private PhotonPoseEstimator photonPoseEstimator1;

    public void PhotonCameraWrapper1() throws IOException {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera1 = new PhotonCamera("front");
        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator1 =
            new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera1, VisionConstants.robotToCam1);
        photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose1(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator1 == null) {
            return Optional.empty();
        }
        photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator1.update();
    }
}
