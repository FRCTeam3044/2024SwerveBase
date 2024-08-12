package frc.robot.autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.autos.reusable.AutoSegment;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.BTrigger;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoTriggers extends AutoSegment {
    public AutoTriggers(String name) {
        super(name);
    }

    public BTrigger nearLocation(Supplier<Translation2d> location, double threshold) {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        return new BTrigger(this.loop, () -> drive.getPose().getTranslation().getDistance(location.get()) < threshold);
    }

    public BTrigger nearLocation(Translation2d location) {
        return nearLocation(() -> location, 1);
    }

    // TODO: MAKE THIS REAL!
    public BTrigger noteDetectedNear(Translation2d location) {
        return nearLocation(location);
    }

    public BTrigger hasNote() {
        BooleanSupplier hasNote = () -> {
            State curState = RobotContainer.stateMachine.getState();
            return curState == State.NOTE_LOADED || curState == State.READY_TO_SHOOT;
        };
        return new BTrigger(this.loop, hasNote);
    }

    public BTrigger readyToShoot() {
        BooleanSupplier readyToShoot = () -> RobotContainer.stateMachine.getState() == State.READY_TO_SHOOT;
        return new BTrigger(this.loop, readyToShoot);
    }

    public BTrigger noNote() {
        BooleanSupplier noNote = () -> {
            State curState = RobotContainer.stateMachine.getState();
            return curState == State.NO_NOTE || curState == State.TARGETING_NOTE;
        };
        return new BTrigger(this.loop, noNote);
    }

    public BTrigger inShootingZone() {
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