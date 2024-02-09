package frc.robot.commands.drive;

import java.util.function.Supplier;
import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TargetRotationController;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

/**
 * Goes to a point while looking at it
 */
public class GoToAndTrackPointCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final TargetRotationController targetRotationController;
    private final ArrayList<Pose2d> target;

    public GoToAndTrackPointCommand(Pose2d target, DriveSubsystem m_robotDrive) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        targetRotationController = new TargetRotationController(target.getX(), target.getY());
        addRequirements(m_robotDrive);
    }

    // Tracks a different target than the path target
    public GoToAndTrackPointCommand(Pose2d target, Pose2d track, DriveSubsystem m_robotDrive) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        targetRotationController = new TargetRotationController(track.getX(), track.getY());
        addRequirements(m_robotDrive);
    }

        public GoToAndTrackPointCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        Pose2d finalTarget = target.get(target.size() - 1);
        targetRotationController = new TargetRotationController(finalTarget.getX(), finalTarget.getY());
        addRequirements(m_robotDrive);
    }

    // Tracks a different target than the path target
    public GoToAndTrackPointCommand(ArrayList<Pose2d> target, Pose2d track, DriveSubsystem m_robotDrive) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        targetRotationController = new TargetRotationController(track.getX(), track.getY());
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        try {
            TrajectoryConfig config = new TrajectoryConfig(0.5,
                    0.5);
            Trajectory myPath = m_robotDrive.pathfinder.generateTrajectory(m_robotDrive.getPose(), target, config);
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);

            Supplier<Double> targetRotSpeed = () -> targetRotationController.calculate(m_robotDrive.getPose(),
                    m_robotDrive.getChassisSpeeds());

            FollowTrajectoryCommand nextCommand = new FollowTrajectoryCommand(myPath, targetRotSpeed, controller,
                    m_robotDrive, m_robotDrive);
            nextCommand.schedule();
        } catch (ImpossiblePathException e) {
            System.out.println("Impossible path, aborting");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
