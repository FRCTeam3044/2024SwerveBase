package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

public class GoToPointSuppliedRotCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final ArrayList<Pose2d> target;
    private FollowTrajectoryCommand nextCommand;
    private Supplier<Rotation2d> rotSupplier;

    public GoToPointSuppliedRotCommand(Pose2d target, DriveSubsystem m_robotDrive, Supplier<Rotation2d> rotSupplier) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        this.rotSupplier = rotSupplier;
    }

    public GoToPointSuppliedRotCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive, Supplier<Rotation2d> rotSupplier) {
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
            TrajectoryConfig config = new TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                    PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());
            Trajectory myPath = m_robotDrive.pathfinder.generateTrajectory(m_robotDrive.getPose(), target, config);
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);
            nextCommand = new FollowTrajectoryCommand(myPath, controller, rotSupplier, m_robotDrive, m_robotDrive);
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
