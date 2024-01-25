package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision;

public class VisionSubsystem extends SubsystemBase {
    DriveSubsystem driveSubsystem = new DriveSubsystem();
    private Vision vision;
    public VisionSubsystem() {
        vision = new Vision();
    }

    @Override
    public void periodic() {
        var visionEst = vision.getEstimatedGlobalPose(0);
        visionEst.ifPresent(
            est -> {
                var estPose = est.estimatedPose.toPose2d();
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs(estPose, 0);

                driveSubsystem.addVisionMeasurement(
                    est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
    }
}
