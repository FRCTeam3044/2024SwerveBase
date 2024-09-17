package frc.robot.statemachine;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class StateCommands {
    // This is a sore spot, since AutoTargetUtils isn't available at startup
    // 1. I will refactor the drive subsystem to use defered command where necessary
    // (like go to point) instead of doing it here
    // 2. I will look at the AllianceUtils class the other people in the Oxplorer
    // discord talked about, they have some good ideas
    public static Command goToSource() {
        Supplier<Command> commandSupplier = () -> {
            Pose2d target = AutoTargetUtils.getSource();
            Pose2d trackTarget = AutoTargetUtils.getSourceTrackTarget();
            return RobotContainer.m_robotDrive.goToAndTrackPoint(target, trackTarget, false, false);
        };
        // Create a new set for dependencies
        Set<Subsystem> dependencies = new HashSet<>();
        dependencies.add(RobotContainer.m_robotDrive);

        return Commands.defer(commandSupplier, dependencies);
    }

    // Also a sore spot - Will look into defered commands more later
    public static Command driveToShootingZone() {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        Supplier<Pose2d> targetSupplier = () -> {
            ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
            if (shootingZone == null) {
                DriverStation.reportError("Unable to retrieve Shooting Zone", null);
                return null;
            }
            Vertex robotPos = new Vertex(drive.getPose());
            if (shootingZone.isInside(robotPos)) {
                return robotPos.asPose2d();
            }
            return shootingZone.calculateNearestPoint(robotPos).asPose2d();
        };

        Supplier<Pose2d> trackPoint = AutoTargetUtils::getShootingTarget;

        return drive.goToAndTrackPoint(targetSupplier, trackPoint, true);
    }

    public static Command pickupNoteAt(Translation2d location) {
        Command driveToNote = RobotContainer.m_robotDrive.goToNote(RobotContainer.m_noteDetection,
                new Pose2d(location, new Rotation2d()), 0.5, false).withName("Pickup Note At");
        return driveToNote.alongWith(RobotContainer.intake.run()).alongWith(RobotContainer.elevator.intake());
    }

    public static Command pickupNote() {
        return Commands.parallel(RobotContainer.m_robotDrive.goToNote(RobotContainer.m_noteDetection, false),
                RobotContainer.intake.run(), RobotContainer.elevator.intake()).withName("SM Pickup Note");
    }
}
