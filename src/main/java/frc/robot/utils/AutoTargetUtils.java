package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PathfindingTargets;
import me.nabdev.pathfinding.structures.Obstacle;

public class AutoTargetUtils {
    public static Pose2d getShootingTarget() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return PathfindingTargets.RED_SHOOTING_TARGET;
        } else {
            return PathfindingTargets.BLUE_SHOOTING_TARGET;
        }
    }

    public static Obstacle getShootingZone() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            return PathfindingTargets.RED_SHOOTING_ZONE;
        } else {
            return PathfindingTargets.BLUE_SHOOTING_ZONE;
        }
    }

    public static Pose2d getSource() {
        Pose2d target;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            target = PathfindingTargets.RED_SOURCE;
        } else {
            target = PathfindingTargets.BLUE_SOURCE;
        }
        return target;
    }
}
