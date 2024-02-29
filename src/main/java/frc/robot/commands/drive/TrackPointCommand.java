package frc.robot.commands.drive;

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

    public TrackPointCommand(DriveSubsystem driveSubsystem, Pose2d target) {
        m_robotDrive = driveSubsystem;
        m_targetRotController = new TargetRotationController(target.getX(), target.getY());
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double inputRot = m_targetRotController.calculate(m_robotDrive.getPose(), m_robotDrive.getChassisSpeeds());
        m_robotDrive.drive(0, 0, inputRot, DriveConstants.kFieldRelative.get(), DriveConstants.kRateLimit.get(), true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
