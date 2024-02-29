package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

public class GoToPointSuppliedRotCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final ArrayList<Pose2d> target;
    private FollowTrajectoryCommand nextCommand;
    private Supplier<Rotation2d> rotSupplier;
    private boolean failed = false;

    public GoToPointSuppliedRotCommand(Pose2d target, DriveSubsystem m_robotDrive, Supplier<Rotation2d> rotSupplier) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        this.rotSupplier = rotSupplier;
    }

    public GoToPointSuppliedRotCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive,
            Supplier<Rotation2d> rotSupplier) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        this.rotSupplier = rotSupplier;
    }

    public GoToPointSuppliedRotCommand(Pose2d target, DriveSubsystem m_robotDrive, Rotation2d rot) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        this.rotSupplier = () -> rot;
    }

    public GoToPointSuppliedRotCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive, Rotation2d rot) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        this.rotSupplier = () -> rot;
    }

    @Override
    public void initialize() {
        try {
            Trajectory myPath = m_robotDrive.generateTrajectory(target);
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);
            nextCommand = new FollowTrajectoryCommand(myPath, controller, rotSupplier, m_robotDrive, m_robotDrive);
            nextCommand.schedule();
            failed = false;
        } catch (ImpossiblePathException e) {
            System.out.println("Impossible path, aborting");
            failed = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (nextCommand != null) {
            return nextCommand.isFinished();
        }
        return failed;
    }

    @Override
    public void end(boolean interrupted) {
        if (nextCommand != null) {
            nextCommand.cancel();
        }
    }
}
