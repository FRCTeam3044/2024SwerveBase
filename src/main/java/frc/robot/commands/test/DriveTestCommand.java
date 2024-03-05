package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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
public class DriveTestCommand extends Command {
    private final CommandXboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    /**
     * Creates a new DriveTestCommand.
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     */
    public DriveTestCommand(RobotContainer robotContainer, DriveSubsystem driveSubsystem,
            CommandXboxController driverController) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
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
        if (m_driverController.getHID().getAButton() || m_driverController.getHID().getYButton()) {
            inputRot = 0;
        }

        m_robotDrive.drive(-inputY, -inputX, -inputRot, false,
                DriveConstants.kRateLimit.get());
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }
}
