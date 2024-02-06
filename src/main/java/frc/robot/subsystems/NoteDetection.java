package frc.robot.subsystems;

import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NoteDetection extends SubsystemBase {
    private Mat homography;
    Timer timer = new Timer();
    private PhotonCamera detector;
    public NoteDetection() {
        detector = new PhotonCamera("detector");
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        MatOfPoint2f cameraPoints = new MatOfPoint2f(
            new Point(621, 538),
            new Point(625, 438),
            new Point(93, 450),
            new Point(1163, 469)
        );

        MatOfPoint2f fieldPoints = new MatOfPoint2f(
            new Point(0.91, 0),
            new Point(1.82, 0),
            new Point(1.82, 0.91),
            new Point(1.82, -0.92)
        );

        Mat test = new Mat(3, 1, CvType.CV_64FC1);
        test.put(0, 0, 93, 450, 1);

        homography = new Mat();
        homography = Calib3d.findHomography(cameraPoints, fieldPoints, Calib3d.RANSAC);

        SmartDashboard.putString("Homography matrix", homography.dump());

        Mat res = new Mat();
        Core.gemm(homography, test, 1, new Mat(), 0, res);
        res.convertTo(res, CvType.CV_64FC1, 1.0 / res.get(2, 0)[0]);

        SmartDashboard.putString("Test result", res.dump());
        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        Pose2d notePose = new Pose2d();
        notePose = new Pose2d(
            currentPose.getX() + res.get(0, 0)[0],
            currentPose.getY() + res.get(1, 0)[0],
            currentPose.getRotation()
        );
        double[] notePoseDouble = { notePose.getX(), notePose.getY(),
            notePose.getRotation().getDegrees() };
        SmartDashboard.putNumberArray("Note", notePoseDouble);
    }

    @Override
    public void periodic() {
        var result = detector.getLatestResult();
        if(!result.hasTargets()) return;
        List<PhotonTrackedTarget> targets = result.getTargets();
        // get midpoint of 0 and 1 detected corners
        var midpoint = targets.get(0).getDetectedCorners().get(0).x + targets.get(0).getDetectedCorners().get(1).x / 2;
        var y = targets.get(0).getDetectedCorners().get(0).y;
        // TODO: Take bounding box from photonvision and convert to field coordinates
        Mat test = new Mat(3, 1, CvType.CV_64FC1);

        test.put(0, 0, midpoint, y, 1);
        Mat res = new Mat();
        Core.gemm(homography, test, 1, new Mat(), 0, res);
        res.convertTo(res, CvType.CV_64FC1, 1.0 / res.get(2, 0)[0]);

        SmartDashboard.putString("Test result", res.dump());
        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        Pose2d notePose = new Pose2d();
        notePose = new Pose2d(
            currentPose.getX() + res.get(0, 0)[0],
            currentPose.getY() + res.get(1, 0)[0],
            currentPose.getRotation()
        );
        double[] notePoseDouble = { notePose.getX(), notePose.getY(),
            notePose.getRotation().getDegrees() };
        SmartDashboard.putNumberArray("Note", notePoseDouble);
        SmartDashboard.putNumber("Time", timer.get());
    }
}
