package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoCommands {
    public static Command driveToShootingZone() {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        Supplier<Pose2d> targetSupplier = () -> {
            ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
            if (shootingZone == null) {
                DriverStation.reportError("Unable to retrieve Shooting Zone", null);
                return null;
            }
            Vertex robotPos = new Vertex(drive.getPose());
            return shootingZone.calculateNearestPoint(robotPos).asPose2d();
        };

        Supplier<Pose2d> trackPoint = AutoTargetUtils::getShootingTarget;

        return drive.goToAndTrackPoint(targetSupplier, trackPoint, true);
    }

    public static Command aimAndShoot() {
        return null;
    }

    public static Command pickupNoteAt(Translation2d location) {
        return null;
    }
}
