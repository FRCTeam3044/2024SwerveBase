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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class NoteDetection extends SubsystemBase {
    public static final ConfigurableParameter<Integer> filterTaps = new ConfigurableParameter<Integer>(3,
            "Filter taps");

    private final ConfigurableParameter<Double> differentNoteThreshold = new ConfigurableParameter<Double>(1.0,
            "Different Note Threshold (meters)");
    private final ConfigurableParameter<Integer> lostFrameThreshold = new ConfigurableParameter<Integer>(50,
            "Lost Note Threshold (Frames)");
    LinearFilter filterNoteX = LinearFilter.movingAverage(filterTaps.get());
    LinearFilter filterNoteY = LinearFilter.movingAverage(filterTaps.get());
    LinearFilter filterRegionNoteX = LinearFilter.movingAverage(filterTaps.get());
    LinearFilter filterRegionNoteY = LinearFilter.movingAverage(filterTaps.get());

    private Mat homography;
    Timer timer = new Timer();
    private PhotonCamera detector;
    public boolean hasNote = false;
    public boolean hasNoteInRegion = false;
    private Pose2d closestPose;
    private Pose2d closestPoseToRegion;
    private double lastDistanceToRobot = Double.MAX_VALUE;
    private double lastDistanceToRegion = Double.MAX_VALUE;
    private int framesSinceRobot;
    private int framesSinceRegion;
    int minX;
    int maxX;
    int minY;
    int midpoint;
    private ArrayList<Pose2d> notePoses = new ArrayList<>();
    private ArrayList<Integer> cameraMidpoints = new ArrayList<>();

    private Pose2d regionPose = null;
    private double regionRadius = 0;

    public NoteDetection() {
        detector = new PhotonCamera("detection");
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        MatOfPoint2f cameraPoints = new MatOfPoint2f(
                new Point(601, 566),
                new Point(289, 568),
                new Point(358, 398),
                new Point(601, 397)
        // new Point(910, 562),
        // new Point(745, 156),
        // new Point(94, 215),
        // new Point(1113, 109),
        // new Point(986, 109),
        // new Point(1118, 48),
        // new Point(1077, 26),
        // new Point(912, 565),
        // new Point(1221, 563),
        // new Point(1196, 283)
        );

        MatOfPoint2f fieldPoints = new MatOfPoint2f(
                new Point(0.9144, 0),
                new Point(0.9144, 0.3048),
                new Point(1.2192, 0.3048),
                new Point(1.2192, 0)
        // new Point(0.9144, -0.3048),
        // new Point(2.1336, -0.3048),
        // new Point(1.8288, 0.9144),
        // new Point(2.7432, 1.2192),
        // new Point(2.7432, 0.9144),
        // new Point(3.3528, 1.524),
        // new Point(3.6576, 1.524),
        // new Point(0.9144, -0.3048),
        // new Point(0.9144, -0.6096),
        // new Point(1.524, -0.9144)
        );

        homography = new Mat();
        homography = Calib3d.findHomography(cameraPoints, fieldPoints, Calib3d.RANSAC);

        SmartDashboard.putString("Homography matrix", homography.dump());
    }

    @Override
    public void periodic() {
        try {
            SmartDashboard.putNumberArray("Closest note", poseToDouble(getClosestNote()));
            SmartDashboard.putBoolean("Has note", hasNote);
            SmartDashboard.putBoolean("Has note in region", hasNoteInRegion);
            // SmartDashboard.putNumber("Frames since robot", framesSinceRobot);
            // SmartDashboard.putNumber("Last Robot Distance", lastDistanceToRobot);
            // SmartDashboard.putNumberArray("Note detector target region",
            // poseToDouble(regionPose));
            notePoses.clear();
            cameraMidpoints.clear();
            if (RobotBase.isSimulation()) {
                closestPose = new Pose2d(2.87, 5.55, new Rotation2d());
                closestPoseToRegion = new Pose2d(2.87, 5.55, new Rotation2d());
                if (closestPose.getTranslation()
                        .getDistance(RobotContainer.m_robotDrive.getPose().getTranslation()) < 3) {
                    hasNote = true;
                    SmartDashboard.putNumberArray("Closest note", poseToDouble(getClosestNote()));
                } else {
                    hasNote = false;
                    SmartDashboard.putNumberArray("Closest note", new double[] {});
                }
                if (regionPose != null) {
                    double d = closestPoseToRegion.getTranslation().getDistance(regionPose.getTranslation());
                    if (d < regionRadius) {
                        hasNoteInRegion = true;
                        SmartDashboard.putNumberArray("Closest note to region", poseToDouble(getClosestNoteToRegion()));
                    } else {
                        hasNoteInRegion = false;
                        SmartDashboard.putNumberArray("Closest note to region", new double[] {});
                    }

                }
                return;
            } else {
                var result = detector.getLatestResult();
                if (!result.hasTargets()) {
                    if (framesSinceRegion > lostFrameThreshold.get()) {
                        hasNoteInRegion = false;
                    }
                    if (framesSinceRobot > lostFrameThreshold.get()) {
                        hasNote = false;
                    }
                    framesSinceRegion++;
                    framesSinceRobot++;
                    return;
                }
                List<PhotonTrackedTarget> targets = result.getTargets();
                for (int i = 0; i < targets.size(); i++) {
                    minX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
                    maxX = (int) targets.get(i).getMinAreaRectCorners().get(0).x;
                    minY = (int) targets.get(i).getMinAreaRectCorners().get(0).y;
                    for (TargetCorner corner : targets.get(i).getMinAreaRectCorners()) {
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
                    int tempMidpoint = (maxX + minX) / 2;

                    notePoses.add(getNotePose(tempMidpoint, minY));
                    cameraMidpoints.add(tempMidpoint);
                    SmartDashboard.putNumberArray("Note pose " + i, poseToDouble(notePoses.get(i)));
                }
            }
            Translation2d robotPose = RobotContainer.m_robotDrive.getPose().getTranslation();
            Pose2d closestRawPose = findClosestNote(null);
            if (closestRawPose == null) {
                hasNote = false;
                return;
            }
            if (regionPose != null) {
                Pose2d closestPoseToRegionTemp = findClosestNote(regionPose);
                if (closestPoseToRegionTemp == null) {
                    framesSinceRegion++;
                    hasNoteInRegion = false;
                } else {
                    double currentDistanceToRegion = robotPose.getDistance(regionPose.getTranslation());

                    if (!hasNote || (currentDistanceToRegion < lastDistanceToRegion
                            && Math.abs(lastDistanceToRegion - currentDistanceToRegion) > differentNoteThreshold
                                    .get())) {
                        lastDistanceToRegion = currentDistanceToRegion;
                        filterRegionNoteX.reset();
                        filterRegionNoteY.reset();
                        closestPose = new Pose2d(filterRegionNoteX.calculate(closestRawPose.getX()),
                                filterRegionNoteY.calculate(closestRawPose.getY()), closestRawPose.getRotation());
                        framesSinceRegion = 0;
                        hasNote = true;
                    } else if (Math.abs(lastDistanceToRegion - currentDistanceToRegion) < differentNoteThreshold
                            .get()) {
                        lastDistanceToRegion = currentDistanceToRegion;
                        closestPose = new Pose2d(filterRegionNoteX.calculate(closestRawPose.getX()),
                                filterRegionNoteY.calculate(closestRawPose.getY()), closestRawPose.getRotation());
                        framesSinceRegion = 0;
                        hasNote = true;
                    } else {
                        framesSinceRegion++;
                        hasNote = false;
                    }

                }
            }

            double currentDistanceToRobot = robotPose.getDistance(closestRawPose.getTranslation());
            if (!hasNote || (currentDistanceToRobot < lastDistanceToRobot
                    && Math.abs(lastDistanceToRobot - currentDistanceToRobot) > differentNoteThreshold.get())) {
                lastDistanceToRobot = currentDistanceToRobot;
                filterNoteX.reset();
                filterNoteY.reset();
                closestPose = new Pose2d(filterNoteX.calculate(closestRawPose.getX()),
                        filterNoteY.calculate(closestRawPose.getY()), closestRawPose.getRotation());
                framesSinceRobot = 0;
                hasNote = true;
            } else if (Math.abs(lastDistanceToRobot - currentDistanceToRobot) < differentNoteThreshold.get()) {
                lastDistanceToRobot = currentDistanceToRobot;
                closestPose = new Pose2d(filterNoteX.calculate(closestRawPose.getX()),
                        filterNoteY.calculate(closestRawPose.getY()), closestRawPose.getRotation());
                framesSinceRobot = 0;
                hasNote = true;
            } else {
                framesSinceRobot++;
                hasNote = false;
            }
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    public Pose2d getClosestNote() {
        return closestPose;
    }

    public Pose2d getClosestNoteToRegion() {
        return closestPoseToRegion;
    }

    public double getClosestNoteDistance() {
        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        Pose2d c = getClosestNote();
        if (c == null) {
            return Double.MAX_VALUE;
        }
        double distance = c.getTranslation().getDistance(currentPose.getTranslation());
        return distance;
    }

    private Pose2d findClosestNote(Pose2d targetRegionPose) {
        Pose2d notePose = null;
        Pose2d currentPose = targetRegionPose == null ? RobotContainer.m_robotDrive.getPose() : regionPose;
        for (int i = 0; i < notePoses.size(); i++) {
            if (i == 0) {
                notePose = notePoses.get(i);
                midpoint = cameraMidpoints.get(i);
            } else {
                if (notePose == null
                        || notePoses.get(i).getTranslation().getDistance(currentPose.getTranslation()) < notePose
                                .getTranslation().getDistance(currentPose.getTranslation())) {
                    notePose = notePoses.get(i);
                    midpoint = cameraMidpoints.get(i);
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
                (1.0 / res.get(2, 0)[0]) * res.get(0, 0)[0] /* + 0.3937 TODO: Move camera offset to constants */
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
        if (pose == null)
            return new double[] {};
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
