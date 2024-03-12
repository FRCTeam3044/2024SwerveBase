package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private FollowTrajectoryCommand m_followCommand;
    private Pose2d originalRobotPose;
    private boolean failed = false;
    private boolean hasTargetRegion;
    private Pose2d targetRegion;
    private double regionRadius;
    private boolean cancelIfNone = true;

    public GoToNoteCommand(DriveSubsystem m_robotDrive, NoteDetection m_noteDetection, boolean cancelIfNone) {
        this.m_robotDrive = m_robotDrive;
        this.m_noteDetection = m_noteDetection;
        hasTargetRegion = false;
        this.cancelIfNone = cancelIfNone;
        targetRotationController = new TargetRotationController(0, 0, false);
    }

    public GoToNoteCommand(DriveSubsystem m_robotDrive, NoteDetection m_noteDetection, Pose2d targetRegion,
            double regionRadius, boolean cancelIfNone) {
        hasTargetRegion = true;
        this.m_robotDrive = m_robotDrive;
        this.m_noteDetection = m_noteDetection;
        this.targetRegion = targetRegion;
        this.regionRadius = regionRadius;
        this.cancelIfNone = cancelIfNone;
        targetRotationController = new TargetRotationController(0, 0, false);

    }

    @Override
    public void initialize() {
        originalRobotPose = m_robotDrive.getPose();
        m_followCommand = null;
        System.out.println("Go to note initialized!!!");
        if (hasTargetRegion) {
            m_noteDetection.setRegion(targetRegion, regionRadius);
            if (m_noteDetection.hasNoteInRegion) {
                Pose2d closestNote = m_noteDetection.getClosestNoteToRegion();
                targetRotationController.setTargetX(closestNote.getX());
                targetRotationController.setTargetY(closestNote.getY());
            } else if (cancelIfNone) {
                failed = true;
            }
        } else {
            if (m_noteDetection.hasNote) {
                Pose2d closestNote = m_noteDetection.getClosestNote();
                targetRotationController.setTargetX(closestNote.getX());
                targetRotationController.setTargetY(closestNote.getY());
            } else if (cancelIfNone) {
                failed = true;
            }

        }
    }

    @Override
    public void execute() {
        if (failed || !m_noteDetection.hasNote || (hasTargetRegion && !m_noteDetection.hasNoteInRegion)) {
            return;
        }
        // System.out.println("Going to note!!!");
        Pose2d notePose = hasTargetRegion ? m_noteDetection.getClosestNoteToRegion() : m_noteDetection.getClosestNote();
        SmartDashboard.putNumberArray("Target Note Pose (StateMachine)",
                new double[] { notePose.getX(), notePose.getY(), 0 });
        targetRotationController.setTargetX(notePose.getX());
        targetRotationController.setTargetY(notePose.getY());

        try {
            Trajectory myPath = m_robotDrive.generateTrajectory(originalRobotPose, notePose);
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