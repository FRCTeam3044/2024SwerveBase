package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

/**
 * Handles the mirroring based on the alliance, which is used in PathingCommand.
 * Please call {@code
 * AllianceUtil.setAlliance()} in disabled periodic of Robot.java.
 */
public class AllianceUtil {
    private static AllianceColor alliance = AllianceColor.UNKNOWN;
    private static Supplier<Pose2d> robotPose;
    private static double fieldLength = Units.feetToMeters(54);
    private static double fieldHeight = Units.feetToMeters(27);
    private static boolean mirroredField = true;

    public enum AllianceColor {
        RED,
        BLUE,
        UNKNOWN;
    }

    /**
     * Set the supplier of the robot's current position. This is taken care of in
     * PathingCommand.
     *
     * @param robotPose Supplies the robot's current position.
     */
    public static void setRobot(Supplier<Pose2d> robotPose) {
        AllianceUtil.robotPose = robotPose;
    }

    /**
     * Set custom field dimensions if you're not using a standard FRC field.
     *
     * @param fieldLength The length of the custom field in meters.
     * @param fieldHeight The height of the custom field in meters.
     */
    public static void setCustomFieldDimensions(double fieldLength, double fieldHeight) {
        AllianceUtil.fieldLength = fieldLength;
        AllianceUtil.fieldHeight = fieldHeight;
    }

    /**
     * Set whether the field design is mirrored or rotated. Defaults to mirrored.
     *
     * @param mirroredField True for mirrored fields, false for rotated fields.
     */
    public static void setCustomFieldDesignType(boolean mirroredField) {
        AllianceUtil.mirroredField = mirroredField;
    }

    /**
     * Sets the current alliance based on driver station data. This should be called
     * in disabled
     * periodic of Robot.java.
     */
    public static void setAlliance() {
        if (DriverStation.getAlliance().isEmpty()) {
            alliance = AllianceColor.UNKNOWN;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = AllianceColor.BLUE;
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            alliance = AllianceColor.RED;
        } else {
            alliance = AllianceColor.UNKNOWN;
        }
    }

    /**
     * Maps a blue pose to the red side based on configured field information.
     *
     * @param bluePose The blue pose to map to red.
     * @return The red pose mapped from the blue pose.
     */
    public static Pose2d mapBluePoseToRed(Pose2d bluePose) {
        if (robotPose == null)
            throw new NullPointerException(
                    "The robot pose supplier is null. Please call AllianceUtil.setRobot() before this method.");
        if (mirroredField) {
            return new Pose2d(
                    fieldLength - bluePose.getX(),
                    bluePose.getY(),
                    new Rotation2d(-(bluePose.getRotation().getRadians() - (Math.PI / 2)) + (Math.PI / 2)));
        } else {
            return new Pose2d(
                    fieldLength - bluePose.getX(),
                    fieldHeight - bluePose.getY(),
                    new Rotation2d(bluePose.getRotation().getRadians() + Math.PI));
        }
    }

    /**
     * @return The current alliance.
     */
    public static AllianceColor getAlliance() {
        return alliance;
    }

    /**
     * @param bluePose The blue pose to reference in order to give the correct pose.
     * @return The correct pose for the alliance.
     */
    public static Pose2d getPoseForAlliance(Pose2d bluePose) {
        Pose2d redPose = mapBluePoseToRed(bluePose);
        if (alliance.equals(AllianceColor.BLUE)) {
            return bluePose;
        } else if (alliance.equals(AllianceColor.RED)) {
            return redPose;
        } else {
            if (robotPose.get().getTranslation().getDistance(bluePose.getTranslation()) < robotPose.get()
                    .getTranslation().getDistance(redPose.getTranslation())) {
                return bluePose;
            } else {
                return redPose;
            }
        }
    }
}