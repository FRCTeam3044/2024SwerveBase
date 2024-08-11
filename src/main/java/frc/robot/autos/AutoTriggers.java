package frc.robot.autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.autos.reusable.AutoSegment;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.BTrigger;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoTriggers extends AutoSegment {
    public Trigger nearLocation(Supplier<Translation2d> location, double threshold) {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        return new BTrigger(this.loop, () -> drive.getPose().getTranslation().getDistance(location.get()) < threshold);
    }

    public Trigger nearLocation(Translation2d location) {
        return nearLocation(() -> location, 1);
    }

    // TODO: MAKE THIS REAL!
    public Trigger noteDetectedNear(Translation2d location) {
        return nearLocation(location);
    }

    public Trigger hasNote() {
        BooleanSupplier hasNote = () -> {
            State curState = RobotContainer.stateMachine.getState();
            return curState == State.NOTE_LOADED || curState == State.READY_TO_SHOOT;
        };
        return new BTrigger(this.loop, hasNote);
    }

    public Trigger readyToShoot() {
        BooleanSupplier readyToShoot = () -> RobotContainer.stateMachine.getState() == State.READY_TO_SHOOT;
        return new BTrigger(this.loop, readyToShoot);
    }

    public Trigger noNote() {
        BooleanSupplier readyToShoot = () -> RobotContainer.stateMachine.getState() == State.NO_NOTE;
        return new BTrigger(this.loop, readyToShoot);
    }

    public Trigger inShootingZone() {
        BooleanSupplier inShootingZone = () -> {
            Vertex robotPos = new Vertex(RobotContainer.m_robotDrive.getPose());
            ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
            if (shootingZone == null) {
                DriverStation.reportError("Unable to retrieve Shooting Zone", null);
                return false;
            }
            return shootingZone.isInside(robotPos);
        };
        return new BTrigger(this.loop, inShootingZone);
    }
}