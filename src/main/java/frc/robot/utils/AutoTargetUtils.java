package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoTargetUtils {
    public static final Pose2d RED_SOURCE = new Pose2d(1, 1, new Rotation2d(0));
    public static final Pose2d BLUE_SOURCE = new Pose2d(1, 15.5, new Rotation2d(0));

    public static final ObstacleGroup BLUE_SHOOTING_ZONE;
    public static final ObstacleGroup RED_SHOOTING_ZONE;

    public static final Pose2d BLUE_SHOOTING_TARGET = new Pose2d(0.1, 5.5478, new Rotation2d());
    public static final Pose2d RED_SHOOTING_TARGET = new Pose2d(16.45, 5.5478, new Rotation2d());

    static {
        Obstacle blueShootingZoneOne = Obstacle.createObstacle(
                new Vertex(1, 3),
                new Vertex(3, 3),
                new Vertex(3, 7),
                new Vertex(1, 7));
        Obstacle blueShootingZoneTwo = Obstacle.createObstacle(
                new Vertex(3, 7),
                new Vertex(3, 5.9),
                new Vertex(5, 5.9),
                new Vertex(5, 7));
        BLUE_SHOOTING_ZONE = new ObstacleGroup(blueShootingZoneOne, blueShootingZoneTwo);

        Obstacle redShootingZoneOne = Obstacle.createObstacle(
                new Vertex(13.55, 3),
                new Vertex(13.55, 7),
                new Vertex(15.55, 7),
                new Vertex(15.55, 3));
        Obstacle redShootingZoneTwo = Obstacle.createObstacle(
                new Vertex(11.55, 7),
                new Vertex(13.55, 7),
                new Vertex(13.55, 5.9),
                new Vertex(11.55, 5.9));
        RED_SHOOTING_ZONE = new ObstacleGroup(redShootingZoneOne, redShootingZoneTwo);

        PathfindingDebugUtils.drawObstacle("Red Shooting Zone", RED_SHOOTING_ZONE);
        PathfindingDebugUtils.drawObstacle("Blue Shooting Zone", BLUE_SHOOTING_ZONE);
    }

    public static Pose2d getShootingTarget() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            if (RobotBase.isSimulation()) {
                return RED_SHOOTING_TARGET;
            }
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return RED_SHOOTING_TARGET;
        } else {
            return BLUE_SHOOTING_TARGET;
        }
    }

    public static ObstacleGroup getShootingZone() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            if (RobotBase.isSimulation()) {
                return RED_SHOOTING_ZONE;
            }
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return RED_SHOOTING_ZONE;
        } else {
            return BLUE_SHOOTING_ZONE;
        }
    }

    /**
     * Get the location where the robot should go to get to the source
     * 
     * @return The source location
     */
    public static Pose2d getSource() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            if (RobotBase.isSimulation()) {
                return RED_SOURCE;
            }
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return RED_SOURCE;
        } else {
            return BLUE_SOURCE;
        }
    }

    /**
     * Get the location where the robot should look at to track the source
     * 
     * @return The source tracking location
     */
    public static Pose2d getSourceTrackTarget() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            if (RobotBase.isSimulation()) {
                return RED_SOURCE;
            }
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return RED_SOURCE;
        } else {
            return BLUE_SOURCE;
        }
    }
}