package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.TargetRotationController;
import me.nabdev.oxconfig.ConfigurableParameter;

/**
 * A command that allows the driver to control the drivebase using the
 * XboxController.
 */
public class DriveAndTrackPointCommand extends Command {
    private final CommandXboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    private final boolean isSimulation;

    private final TargetRotationController m_targetRotController;

    /**
     * Creates a new ManualDriveCommand.
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     */
    public DriveAndTrackPointCommand(DriveSubsystem driveSubsystem, CommandXboxController driverController) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
        isSimulation = RobotBase.isSimulation();
        m_targetRotController = new TargetRotationController(0, 0);
        new ConfigurableParameter<Double>(1.0, "Drive & Track Target X",
                m_targetRotController::setTargetX);
        new ConfigurableParameter<Double>(0.0, "Drive & Track Target Y",
                m_targetRotController::setTargetY);
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double inputX = MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get());
        double inputY = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get());
        double inputRot = m_targetRotController.calculate(m_robotDrive.getPose(), m_robotDrive.getChassisSpeeds());

        // TODO: Needs to command rotation speed, right now input rot is expected to be a controller input
        if (isSimulation) {
            m_robotDrive.drive(inputX, -inputY, inputRot, DriveConstants.kFieldRelative.get(), DriveConstants.kRateLimit.get());
        } else {
            m_robotDrive.drive(-inputY, -inputX, inputRot, DriveConstants.kFieldRelative.get(), DriveConstants.kRateLimit.get());
        }
    }
}
