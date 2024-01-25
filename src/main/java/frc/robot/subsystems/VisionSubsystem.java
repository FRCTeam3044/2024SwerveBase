package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vision;

public class VisionSubsystem extends SubsystemBase {
    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private Vision vision;
    public VisionSubsystem() {
        vision = new Vision();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < Constants.VisionConstants.activeCameras.length; i++) {
            final int camera = i;
            var visionEst = vision.getEstimatedGlobalPose(i);
            visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs(estPose, camera);

                    driveSubsystem.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
        }
    }
}
