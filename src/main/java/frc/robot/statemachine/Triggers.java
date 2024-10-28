package frc.robot.statemachine;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class Triggers {
    public static BooleanSupplier nearLocation(Supplier<Translation2d> location, double threshold) {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        return () -> drive.getPose().getTranslation().getDistance(location.get()) < threshold;
    }

    public static BooleanSupplier nearLocation(Translation2d location, double threshold) {
        return nearLocation(() -> location, threshold);
    }

    // TODO: MAKE THIS REAL!
    public static BooleanSupplier noteDetectedNear(Translation2d location) {
        return () -> {
            RobotContainer.m_noteDetection.setRegion(new Pose2d(location, new Rotation2d()), 0.5);
            return RobotContainer.m_noteDetection.hasNoteInRegion;
        };
    }

    // TODO: Make this real!
    public static BooleanSupplier readyToShoot() {
        BooleanSupplier readyToShoot = () -> true;
        return readyToShoot;
    }

    // Thinking about making this a pattern, where conditions are static and
    // triggers are instance methods
    // To use for state entrance conditions
    public static BooleanSupplier inShootingZone() {
        return () -> {
            Vertex robotPos = new Vertex(RobotContainer.m_robotDrive.getPose());
            ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
            if (shootingZone == null) {
                DriverStation.reportError("Unable to retrieve Shooting Zone", null);
                return false;
            }
            return shootingZone.isInside(robotPos);
        };
    }
}