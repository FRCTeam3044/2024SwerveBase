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
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NoteDetection extends SubsystemBase {
    private Mat homography;
    Timer timer = new Timer();
    private PhotonCamera detector;
    Pose2d notePose = new Pose2d();
    int minX;
    int maxX;
    int minY;
    public NoteDetection() {
        detector = new PhotonCamera("detection");
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        MatOfPoint2f cameraPoints = new MatOfPoint2f(
            new Point(501,638),
            new Point(789,643),
            new Point(755,587),
            new Point(524,579)
        );

        MatOfPoint2f fieldPoints = new MatOfPoint2f(
            new Point(0.61595,0.08255),
            new Point(0.61595,-0.08255),
            new Point(0.78105,-0.08255),
            new Point(0.78105,0.08255)
        );

        // Mat test = new Mat(3, 1, CvType.CV_64FC1);
        // test.put(0, 0, 93, 450, 1);

        homography = new Mat();
        homography = Calib3d.findHomography(cameraPoints, fieldPoints, Calib3d.RANSAC);

        SmartDashboard.putString("Homography matrix", homography.dump());

        // Mat res = new Mat();
        // Core.gemm(homography, test, 1, new Mat(), 0, res);
        // res.convertTo(res, CvType.CV_64FC1, 1.0 / res.get(2, 0)[0]);

        // SmartDashboard.putString("Test result", res.dump());
        // Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        // Pose2d notePose = new Pose2d();
        // notePose = new Pose2d(
        //     currentPose.getX() + res.get(0, 0)[0],
        //     currentPose.getY() + res.get(1, 0)[0],
        //     currentPose.getRotation()
        // );
        // double[] notePoseDouble = { notePose.getX(), notePose.getY(),
        //     notePose.getRotation().getDegrees() };
        // SmartDashboard.putNumberArray("Note", notePoseDouble);
    }

    @Override
    public void periodic() {
        var result = detector.getLatestResult();
        if(!result.hasTargets()) {
            return;
        }
        List<PhotonTrackedTarget> targets = result.getTargets();
        // System.out.println("before the fire");
        minX = (int) targets.get(0).getMinAreaRectCorners().get(0).x;
        maxX = (int) targets.get(0).getMinAreaRectCorners().get(0).x;
        minY = (int) targets.get(0).getMinAreaRectCorners().get(0).y;
        for(PhotonTrackedTarget target : targets) {
            for(TargetCorner corner : target.getMinAreaRectCorners()) {
                // if(minX == 9000 && maxX == 0 && minY == 9000) {
                //     // System.out.println("values are zero");
                //     minX = (int) corner.x;
                //     maxX = (int) corner.x;
                //     minY = (int) corner.y;
                // } else {
                    if(corner.x < minX) {
                        minX = (int) corner.x;
                    }
                    if(corner.x > maxX) {
                        maxX = (int) corner.x;
                    }
                    if(corner.y < minY) {
                        minY = (int) corner.y;
                    // }
                }
            }
        }
        // System.out.println("fire has been put out");
        var midpoint = (maxX + minX) / 2;
        SmartDashboard.putNumber("minX", minX);
        SmartDashboard.putNumber("maxX", maxX);
        SmartDashboard.putNumber("minY", minY);
        SmartDashboard.putNumber("midPoint", midpoint);
        
        Mat noteScreenSpace = new Mat(3, 1, CvType.CV_64FC1);

        noteScreenSpace.put(0, 0, midpoint, minY, 1);
        Mat res = new Mat();
        Core.gemm(homography, noteScreenSpace, 1, new Mat(), 0, res);
        res.convertTo(res, CvType.CV_64FC1, 1.0 / res.get(2, 0)[0]);

        SmartDashboard.putString("Test result", res.dump());
        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        notePose = new Pose2d(
            currentPose.getX() + (1.0 / res.get(2, 0)[0]) * res.get(0, 0)[0],
            currentPose.getY() + (1.0 / res.get(2, 0)[0]) * res.get(1, 0)[0],
            currentPose.getRotation()
        );
        double[] notePoseDouble = { notePose.getX(), notePose.getY(),
            notePose.getRotation().getDegrees() };
        SmartDashboard.putNumberArray("Note pose", notePoseDouble);
    }
}
