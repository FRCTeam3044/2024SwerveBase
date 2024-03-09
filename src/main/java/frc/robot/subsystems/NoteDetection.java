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
import frc.robot.Constants.DetectorConstants;

public class NoteDetection extends SubsystemBase {
    LinearFilter filterNoteX = LinearFilter.movingAverage(DetectorConstants.filterTaps.get());
    LinearFilter filterNoteY = LinearFilter.movingAverage(DetectorConstants.filterTaps.get());

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

        homography = new Mat();
        homography = Calib3d.findHomography(DetectorConstants.cameraPointsArray, DetectorConstants.fieldPointsArray, Calib3d.RANSAC);

        SmartDashboard.putString("Homography matrix", homography.dump());
    }

    @Override
    public void periodic() {
        notePoses.clear();
        if (RobotBase.isSimulation()) {
            hasNote = true;
            closestPose = new Pose2d(13.68, 7, new Rotation2d());
            if (closestPose.getTranslation().getDistance(RobotContainer.m_robotDrive.getPose().getTranslation()) < 1) {
                notePoses.add(closestPose);
                SmartDashboard.putNumberArray("Closest note", poseToDouble(getClosestNote()));
            }
        } else {
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
        }
        Pose2d closestRawPose = findClosestNote(null);
        if (closestRawPose == null) {
            return;
        }
        if (regionPose != null) {
            Pose2d closestPoseToRegionTemp = findClosestNote(regionPose);
            if (closestPoseToRegionTemp == null) {
                hasNoteInRegion = false;
            } else {
                hasNoteInRegion = true;
                closestPoseToRegion = new Pose2d(
                        filterNoteX.calculate(closestPoseToRegionTemp.getX()),
                        filterNoteY.calculate(closestPoseToRegionTemp.getY()),
                        closestPoseToRegionTemp.getRotation());
            }
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

    private Pose2d findClosestNote(Pose2d targetRegionPose) {
        Pose2d notePose = null;
        Pose2d currentPose = targetRegionPose == null ? RobotContainer.m_robotDrive.getPose() : regionPose;
        for (int i = 0; i < notePoses.size(); i++) {
            if (i == 0) {
                notePose = notePoses.get(i);
            } else {
                if (notePose == null
                        || notePoses.get(i).getTranslation().getDistance(currentPose.getTranslation()) < notePose
                                .getTranslation().getDistance(currentPose.getTranslation())) {
                    notePose = notePoses.get(i);
                }
            }
        }
        if (targetRegionPose != null) {
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
                (1.0 / res.get(2, 0)[0]) * res.get(0, 0)[0] + DetectorConstants.cameraOffset + DetectorConstants.noteCenterDist,
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
