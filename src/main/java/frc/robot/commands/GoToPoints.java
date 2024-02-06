package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

public class GoToPoints extends Command {
    private final DriveSubsystem m_robotDrive;
    private final ArrayList<Pose2d> target;
    private FollowTrajectory nextCommand;

    public GoToPoints(Pose2d target, DriveSubsystem m_robotDrive) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
    }

    public GoToPoints(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
    }

    @Override
    public void initialize() {
        try {
            TrajectoryConfig config = new TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                    PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());
            Trajectory myPath = m_robotDrive.pathfinder.generateTrajectory(m_robotDrive.getPose(), target, config);
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);

            Supplier<Rotation2d> targetRotSupplier = () -> Rotation2d.fromDegrees(0);
            nextCommand = new FollowTrajectory(myPath, controller, targetRotSupplier, m_robotDrive,
                    m_robotDrive);
            nextCommand.schedule();
        } catch (ImpossiblePathException e) {
            System.out.println("Impossible path, aborting");
        }
    }

    @Override
    public boolean isFinished() {
        return nextCommand.isFinished();
    }
}
