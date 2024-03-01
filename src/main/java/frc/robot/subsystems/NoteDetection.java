package frc.robot.subsystems;

import java.util.ArrayList;
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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class NoteDetection extends SubsystemBase {
    public static final ConfigurableParameter<Integer> filterTaps = new ConfigurableParameter<Integer>(3,
            "Filter taps");
    LinearFilter filterNoteX = LinearFilter.movingAverage(filterTaps.get());
    LinearFilter filterNoteY = LinearFilter.movingAverage(filterTaps.get());

    private Mat homography;
    Timer timer = new Timer();
    private PhotonCamera detector;
    public boolean hasNote = false;
    public boolean hasNoteInRegion = false;
    private Pose2d closestPose;
    private Pose2d closestPoseToRegion;
    int minX;
    int maxX;
    int minY;
    private ArrayList<Pose2d> notePoses = new ArrayList<>();

    private Pose2d regionPose = null;
    private double regionRadius = 0;

    public NoteDetection() {
        detector = new PhotonCamera("detection");
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        MatOfPoint2f cameraPoints = new MatOfPoint2f(
                new Point(501, 638),
                new Point(789, 643),
                new Point(755, 587),
                new Point(524, 579));

        MatOfPoint2f fieldPoints = new MatOfPoint2f(
                new Point(0.61595, 0.08255),
                new Point(0.61595, -0.08255),
                new Point(0.78105, -0.08255),
                new Point(0.78105, 0.08255));

        homography = new Mat();
        homography = Calib3d.findHomography(cameraPoints, fieldPoints, Calib3d.RANSAC);

        SmartDashboard.putString("Homography matrix", homography.dump());
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            hasNote = true;
            closestPose = new Pose2d(16.05, 2, new Rotation2d());
            SmartDashboard.putNumberArray("Closest note", poseToDouble(getClosestNote()));
            return;
        }
        notePoses.clear();

        var result = detector.getLatestResult();
        if (!result.hasTargets()) {
            hasNote = false;
            hasNoteInRegion = false;
            return;
        }
        hasNote = true;
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (int i = 0; i < targets.size(); i++) {
            minX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
            maxX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
            minY = (int) targets.get(i).getMinAreaRectCorners().get(0).y;
            for (PhotonTrackedTarget target : targets) {
                for (TargetCorner corner : target.getMinAreaRectCorners()) {
                    if (corner.x < minX) {
                        minX = (int) corner.x;
                    }
                    if (corner.x > maxX) {
                        maxX = (int) corner.x;
                    }
                    if (corner.y < minY) {
                        minY = (int) corner.y;
                    }
                }
            }
            var midpoint = (maxX + minX) / 2;

            notePoses.add(getNotePose(midpoint, minY));
            SmartDashboard.putNumberArray("Note pose " + i, poseToDouble(notePoses.get(i)));
        }
        Pose2d closestRawPose = findClosestNote(null);
        Pose2d closestPoseToRegion = findClosestNote(regionPose);
        if (closestPoseToRegion == null) {
            hasNoteInRegion = false;
        } else {
            hasNoteInRegion = true;
            closestPoseToRegion = new Pose2d(
                    filterNoteX.calculate(closestPoseToRegion.getX()),
                    filterNoteY.calculate(closestPoseToRegion.getY()),
                    closestPoseToRegion.getRotation());
        }
        closestPose = new Pose2d(
                filterNoteX.calculate(closestRawPose.getX()),
                filterNoteY.calculate(closestRawPose.getY()),
                closestRawPose.getRotation());

        SmartDashboard.putNumberArray("Closest note", poseToDouble(getClosestNote()));
        SmartDashboard.putBoolean("Has note", hasNote);
        SmartDashboard.putBoolean("Has note in region", hasNoteInRegion);
    }

    public Pose2d getClosestNote() {
        return closestPose;
    }

    public Pose2d getClosestNoteToRegion() {
        return closestPoseToRegion;
    }

    public double getClosestNoteDistance() {
        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        double distance = getClosestNote().getTranslation().getDistance(currentPose.getTranslation());
        return distance;
    }

    private Pose2d findClosestNote(Pose2d regionPose) {
        Pose2d notePose = new Pose2d();
        Pose2d currentPose = regionPose == null ? RobotContainer.m_robotDrive.getPose() : regionPose;
        for (int i = 0; i < notePoses.size(); i++) {
            if (i == 0) {
                notePose = notePoses.get(i);
            } else {
                if (notePoses.get(i).getTranslation().getDistance(currentPose.getTranslation()) < notePose
                        .getTranslation().getDistance(currentPose.getTranslation())) {
                    notePose = notePoses.get(i);
                }
            }
        }
        if (regionPose != null) {
            if (notePose.getTranslation().getDistance(currentPose.getTranslation()) > regionRadius) {
                return null;
            }
        }

        return notePose;
    }

    private Pose2d getNotePose(int x, int y) {
        Mat noteScreenSpace = new Mat(3, 1, CvType.CV_64FC1);

        noteScreenSpace.put(0, 0, x, y, 1);
        Mat res = new Mat();
        Core.gemm(homography, noteScreenSpace, 1, new Mat(), 0, res);
        res.convertTo(res, CvType.CV_64FC1, 1.0 / res.get(2, 0)[0]);

        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        Pose2d robotRelativeNotePose = new Pose2d(
                (1.0 / res.get(2, 0)[0]) * res.get(0, 0)[0] + 0.3937 /* TODO: Move camera offset to constants */
                        + 0.1778 /* Note center dist */,
                (1.0 / res.get(2, 0)[0]) * res.get(1, 0)[0],
                new Rotation2d());
        // Translate the note pose to the field (rotate by the robot's rotation)
        double robotRotationRad = currentPose.getRotation().getRadians();
        double noteX = robotRelativeNotePose.getX();
        double noteY = robotRelativeNotePose.getY();

        Pose2d rotatedNotePose = new Pose2d(
                noteX * Math.cos(robotRotationRad) - noteY * Math.sin(robotRotationRad),
                noteX * Math.sin(robotRotationRad) + noteY * Math.cos(robotRotationRad),
                new Rotation2d());

        Pose2d notePose = new Pose2d(
                rotatedNotePose.getX() + currentPose.getX(),
                rotatedNotePose.getY() + currentPose.getY(),
                new Rotation2d());

        return notePose;
    }

    private double[] poseToDouble(Pose2d pose) {
        double[] poseDouble = { pose.getX(), pose.getY(), pose.getRotation().getDegrees() };
        return poseDouble;
    }

    public void setRegion(Pose2d pose, double radius) {
        regionPose = pose;
        regionRadius = radius;
    }

    public void clearRegion() {
        regionPose = null;
        regionRadius = 0;
    }
}
