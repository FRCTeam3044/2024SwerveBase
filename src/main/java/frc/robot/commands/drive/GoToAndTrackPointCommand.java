package frc.robot.commands.drive;

import java.util.function.Supplier;
import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
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
    private FollowTrajectoryCommand m_followTrajectoryCommand;
    private boolean failed = false;
    private boolean noObstacles = false;

    public GoToAndTrackPointCommand(Pose2d target, DriveSubsystem m_robotDrive, boolean flipped) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        targetRotationController = new TargetRotationController(target.getX(), target.getY(), flipped);
    }

    // Tracks a different target than the path target
    public GoToAndTrackPointCommand(Pose2d target, Pose2d track, DriveSubsystem m_robotDrive, boolean flipped,
            boolean noObstacles) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        // this.noObstacles = noObstacles;
        targetRotationController = new TargetRotationController(track.getX(), track.getY(), flipped);

    }

    public GoToAndTrackPointCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive, boolean flipped) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        Pose2d finalTarget = target.get(target.size() - 1);
        targetRotationController = new TargetRotationController(finalTarget.getX(), finalTarget.getY(), flipped);
    }

    // Tracks a different target than the path target
    public GoToAndTrackPointCommand(ArrayList<Pose2d> target, Pose2d track, DriveSubsystem m_robotDrive,
            boolean flipped, boolean noObstacles) {
        // this.noObstacles = noObstacles;
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        targetRotationController = new TargetRotationController(track.getX(), track.getY(), flipped);

    }

    @Override
    public void initialize() {
        try {
            Trajectory myPath;
            if (noObstacles) {
                myPath = m_robotDrive.generateTrajectoryNoAvoidance(m_robotDrive.getPose(), target.get(0));
            } else {
                myPath = m_robotDrive.generateTrajectory(target);
            }
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);

            Supplier<Double> targetRotSpeed = () -> targetRotationController.calculate(m_robotDrive.getPose(),
                    m_robotDrive.getChassisSpeeds());

            m_followTrajectoryCommand = new FollowTrajectoryCommand(myPath, targetRotSpeed, controller,
                    m_robotDrive, m_robotDrive);
            m_followTrajectoryCommand.schedule();
            failed = false;
        } catch (ImpossiblePathException e) {
            System.out.println("Impossible path, aborting");
            failed = true;
        }
    }

    @Override
    public void execute() {
        // System.out.println(getName() + " Running");
    }

    @Override
    public boolean isFinished() {
        if (m_followTrajectoryCommand == null) {
            return failed;
        }
        return m_followTrajectoryCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + " Ended");
        if (m_followTrajectoryCommand != null) {
            m_followTrajectoryCommand.cancel();
        }
    }

    @Override
    public void cancel() {
        // CommandScheduler.getInstance().cancel(this);
        if (m_followTrajectoryCommand != null) {
            m_followTrajectoryCommand.cancel();
        }
    }
}
