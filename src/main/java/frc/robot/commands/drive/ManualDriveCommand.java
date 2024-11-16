package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/**
 * A command that allows the driver to control the drivebase using the
 * XboxController.
 */
public class ManualDriveCommand extends Command {
    private final CommandXboxController m_driverController;
    private final DriveSubsystem m_robotDrive;
    private final RobotContainer m_robotContainer;
    private final boolean isSimulation;

    /**
     * Creates a new ManualDriveCommand.
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     */
    public ManualDriveCommand(RobotContainer robotContainer, DriveSubsystem driveSubsystem,
            CommandXboxController driverController) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
        m_robotContainer = robotContainer;
        isSimulation = RobotBase.isSimulation();
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double inputX = MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get());
        double inputY = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get());
        double inputRot = MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get());
        boolean slow = m_driverController.getRightTriggerAxis() > 0.5;
        inputX = inputX * inputX * Math.signum(inputX);
        inputY = inputY * inputY * Math.signum(inputY);
        inputRot = inputRot * inputRot * Math.signum(inputRot);

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            inputY *= -1;
            inputX *= -1;
        }

        if (isSimulation) {
            m_robotDrive.drive(inputX, -inputY, -inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), false, slow);
        } else {
            m_robotDrive.drive(inputY, inputX, -inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), false, slow);
        }
    }
}
