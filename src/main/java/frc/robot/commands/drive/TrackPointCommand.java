package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.TargetRotationController;

/**
 * A command that allows the driver to control the drivebase using the
 * XboxController.
 */
public class TrackPointCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final TargetRotationController m_targetRotController;
    private Pose2d lastTarget;
    private final Supplier<Pose2d> targetSupplier;

    public TrackPointCommand(DriveSubsystem driveSubsystem, Pose2d target, boolean flipped) {
        lastTarget = target;
        targetSupplier = () -> target;
        m_robotDrive = driveSubsystem;
        m_targetRotController = new TargetRotationController(target.getX(), target.getY(), flipped);
        addRequirements(m_robotDrive);
    }

    public TrackPointCommand(Supplier<Pose2d> target, DriveSubsystem driveSubsystem, boolean flipped) {
        targetSupplier = target;
        try {
            lastTarget = target.get();
        } catch (Throwable e) {
            e.printStackTrace();
        }
        m_robotDrive = driveSubsystem;
        m_targetRotController = new TargetRotationController(lastTarget.getX(), lastTarget.getY(), flipped);
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        try {
            if (lastTarget != targetSupplier.get()) {
                m_targetRotController.setTargetX(targetSupplier.get().getX());
                m_targetRotController.setTargetY(targetSupplier.get().getY());
                lastTarget = targetSupplier.get();
            }
        } catch (Throwable e) {
            e.printStackTrace();
        }
        double inputRot = m_targetRotController.calculate(m_robotDrive.getPose(), m_robotDrive.getChassisSpeeds());
        m_robotDrive.drive(0, 0, inputRot, DriveConstants.kFieldRelative.get(), DriveConstants.kRateLimit.get(), true,
                false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
