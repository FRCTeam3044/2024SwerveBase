package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.pathfinding.autos.AutoBoolean;

public class RobotWithinRadius implements AutoBoolean {
    private DriveSubsystem m_robotDrive;
    private Pose2d m_target;
    private double radius;

    public RobotWithinRadius(DriveSubsystem robotDrive, Pose2d target, double radius) {
        m_robotDrive = robotDrive;
        m_target = target;
        this.radius = radius;
    }

    @Override
    public BooleanSupplier getSupplier(BooleanSupplier... children) {
        return this::inRange;
    }

    private boolean inRange() {
        return m_robotDrive.getPose().getTranslation().getDistance(m_target.getTranslation()) < radius;
    }
}
