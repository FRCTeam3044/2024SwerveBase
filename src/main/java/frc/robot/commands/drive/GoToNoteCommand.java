package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.utils.TargetRotationController;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

/**
 * Goes to a point while looking at it
 */
public class GoToNoteCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final NoteDetection m_noteDetection;
    private final TargetRotationController targetRotationController;
    private TrajectoryConfig config;
    private FollowTrajectoryCommand m_followCommand;
    private Pose2d originalRobotPose;
    private boolean failed = false;

    public GoToNoteCommand(DriveSubsystem m_robotDrive, NoteDetection m_noteDetection) {
        this.m_robotDrive = m_robotDrive;
        this.m_noteDetection = m_noteDetection;
        if (m_noteDetection.hasNote) {
            Pose2d closestNote = m_noteDetection.getClosestNote();
            targetRotationController = new TargetRotationController(closestNote.getX(),
                    closestNote.getY());
        } else {
            targetRotationController = new TargetRotationController(0, 0);
        }
    }

    @Override
    public void initialize() {
        config = new TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());
        originalRobotPose = m_robotDrive.getPose();
        m_followCommand = null;
    }

    @Override
    public void execute() {
        if (!m_noteDetection.hasNote) {
            return;
        }
        Pose2d notePose = m_noteDetection.getClosestNote();
        targetRotationController.setTargetX(notePose.getX());
        targetRotationController.setTargetY(notePose.getY());

        try {
            Trajectory myPath = m_robotDrive.pathfinder.generateTrajectory(originalRobotPose, notePose, config);
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);

            Supplier<Double> targetRotSpeed = () -> targetRotationController.calculate(m_robotDrive.getPose(),
                    m_robotDrive.getChassisSpeeds());

            if (m_followCommand == null) {
                m_followCommand = new FollowTrajectoryCommand(myPath, targetRotSpeed, controller,
                        m_robotDrive, m_robotDrive);
                m_followCommand.schedule();
            } else {
                m_followCommand.setTrajectory(myPath);
            }
            failed = false;
        } catch (ImpossiblePathException e) {
            System.out.println("Impossible path, aborting");
            failed = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_followCommand == null) {
            return failed;
        }
        return m_followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_followCommand != null) {
            m_followCommand.cancel();
        }
    }
}