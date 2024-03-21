package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Vision;

public class VisionSubsystem extends SubsystemBase {
    private Vision vision;
    private boolean hasTargets;

    public VisionSubsystem() {

        vision = new Vision();
        RobotContainer.m_robotDrive.addVisionMeasurement(new Pose2d(0, 0, new Rotation2d(0)), 0);
    }

    @Override
    public void periodic() {
        hasTargets = false;
        for (int i = 0; i < Constants.VisionConstants.activeCameras.length; i++) {
            final int camera = i;
            Optional<EstimatedRobotPose> visionEst = vision.getEstimatedGlobalPose(i);
            if (visionEst.isPresent()) {
                hasTargets = true;
            }
            visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();
                        // SmartDashboard.putNumberArray("est robot", poseToDouble(estPose));
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = vision.getEstimationStdDevs(estPose, camera);

                        RobotContainer.m_robotDrive.addVisionMeasurement(
                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }

        double[] pose = poseToDouble(RobotContainer.m_robotDrive.getPose());

        // Is being logged on field
        SmartDashboard.putNumberArray("Robot Pose", pose);
    }

    private double[] poseToDouble(Pose2d pose) {
        double[] poseDouble = { pose.getX(), pose.getY(), pose.getRotation().getDegrees() };
        return poseDouble;
    }

    public boolean hasTargets() {
        return hasTargets;
    }
}
