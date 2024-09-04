package frc.robot.statemachine;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.statemachine.CrescendoStateMachine.States;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.BTrigger;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public abstract class StateTriggers extends State {
    public StateTriggers(StateMachine stateMachine, Enum<?> name) {
        super(stateMachine, name);
    }

    public BTrigger nearLocation(Supplier<Translation2d> location, double threshold) {
        DriveSubsystem drive = RobotContainer.m_robotDrive;
        return new BTrigger(this.loop, () -> drive.getPose().getTranslation().getDistance(location.get()) < threshold);
    }

    public BTrigger nearLocation(Translation2d location) {
        return nearLocation(() -> location, 1.5);
    }

    // TODO: MAKE THIS REAL!
    public BTrigger noteDetectedNear(Translation2d location) {
        return new BTrigger(() -> {
            RobotContainer.m_noteDetection.setRegion(new Pose2d(location, new Rotation2d()), 0.5);
            return RobotContainer.m_noteDetection.hasNoteInRegion;
        });
    }

    public BTrigger hasNote() {
        BooleanSupplier hasNote = () -> {
            StateMachine sm = CrescendoStateMachine.getInstance();
            return sm.is(States.HAS_NOTE) || sm.is(States.READY_TO_SHOOT) || sm.is(States.SHOOTING);
        };
        return new BTrigger(this.loop, hasNote);
    }

    public BTrigger readyToShoot() {
        BooleanSupplier readyToShoot = () -> CrescendoStateMachine
                .getInstance().currentState.name == States.READY_TO_SHOOT;
        return new BTrigger(this.loop, readyToShoot);
    }

    public BTrigger noNote() {
        BooleanSupplier noNote = () -> {
            StateMachine sm = CrescendoStateMachine.getInstance();
            return sm.is(States.NO_NOTE);
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