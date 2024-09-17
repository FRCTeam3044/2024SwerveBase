package frc.robot.statemachine;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.SmartEventLoop;
import frc.robot.statemachine.reusable.SmartTrigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class Triggers {
    private SmartEventLoop loop = new SmartEventLoop();

    public Triggers(SmartEventLoop loop) {
        this.loop = loop;
    }

    public SmartTrigger nearLocationTrg(Supplier<Translation2d> location, double threshold) {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        return new SmartTrigger(this.loop,
                () -> drive.getPose().getTranslation().getDistance(location.get()) < threshold);
    }

    public SmartTrigger nearLocationTrg(Translation2d location) {
        return nearLocationTrg(() -> location, 1.5);
    }

    // TODO: MAKE THIS REAL!
    public SmartTrigger noteDetectedNearTrg(Translation2d location) {
        return new SmartTrigger(this.loop, () -> {
            RobotContainer.m_noteDetection.setRegion(new Pose2d(location, new Rotation2d()), 0.5);
            return RobotContainer.m_noteDetection.hasNoteInRegion;
        });
    }

    // TODO: Make this real!
    public SmartTrigger hasNoteTrg() {
        BooleanSupplier hasNote = () -> true;
        return new SmartTrigger(this.loop, hasNote);
    }

    // TODO: Make this real!
    public SmartTrigger readyToShootTrg() {
        BooleanSupplier readyToShoot = () -> true;
        return new SmartTrigger(this.loop, readyToShoot);
    }

    // TODO: Make this real!
    public SmartTrigger noNoteTrg() {
        BooleanSupplier noNote = () -> true;
        return new SmartTrigger(this.loop, noNote);
    }

    public SmartTrigger inShootingZoneTrg() {
        return new SmartTrigger(this.loop, inShootingZone());
    }

    // Thinking about making this a pattern, where conditions are static and triggers are instance methods
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