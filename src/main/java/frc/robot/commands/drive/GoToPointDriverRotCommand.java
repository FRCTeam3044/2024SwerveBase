package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

public class GoToPointDriverRotCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final ArrayList<Pose2d> target;
    private FollowTrajectoryCommand nextCommand;
    private CommandXboxController m_driverController;
    private boolean failed = false;

    public GoToPointDriverRotCommand(Pose2d target, DriveSubsystem m_robotDrive,
            CommandXboxController m_driverController) {
        this.target = new ArrayList<Pose2d>();
        this.target.add(target);
        this.m_robotDrive = m_robotDrive;
        this.m_driverController = m_driverController;
    }

    public GoToPointDriverRotCommand(ArrayList<Pose2d> target, DriveSubsystem m_robotDrive,
            CommandXboxController m_driverController) {
        this.target = target;
        this.m_robotDrive = m_robotDrive;
        this.m_driverController = m_driverController;
    }

    @Override
    public void initialize() {
        try {
            Trajectory myPath = m_robotDrive.pathfinder.generateTrajectory(m_robotDrive.getPose(), target,
                    m_robotDrive.getTrajectoryConfig());
            m_robotDrive.field.getObject("Path").setTrajectory(myPath);

            HolonomicDriveController controller = new HolonomicDriveController(
                    PathfindingConstants.kPathfindingXController, PathfindingConstants.kPathfindingYController,
                    PathfindingConstants.kPathfindingThetaController);
            Supplier<Double> targetRotSpeedSupplier = () -> m_robotDrive.rotSpeedFromJoystick(
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get()), true);
            nextCommand = new FollowTrajectoryCommand(myPath, targetRotSpeedSupplier, controller, m_robotDrive,
                    m_robotDrive);
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
