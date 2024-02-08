package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

/**
 * A command that allows the driver to control the drivebase using the
 * XboxController.
 */
public class ManualDriveCommand extends Command {
    private final CommandXboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    private final ConfigurableParameter<Boolean> m_fieldRelative = new ConfigurableParameter<Boolean>(true,
            "Field Relative");
    private final ConfigurableParameter<Boolean> m_rateLimit = new ConfigurableParameter<Boolean>(true, "Rate Limit");

    private final boolean isSimulation;

    /**
     * Creates a new ManualDriveCommand.
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     */
    public ManualDriveCommand(DriveSubsystem driveSubsystem, CommandXboxController driverController) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
        isSimulation = RobotBase.isSimulation();
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double inputX = MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get());
        double inputY = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get());
        double inputRot = MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get());

        if (isSimulation) {
            m_robotDrive.drive(inputY, inputX, -inputRot, m_fieldRelative.get(), m_rateLimit.get());
        } else {
            m_robotDrive.drive(inputX, -inputY, -inputRot, m_fieldRelative.get(), m_rateLimit.get());
        }
    }
}
