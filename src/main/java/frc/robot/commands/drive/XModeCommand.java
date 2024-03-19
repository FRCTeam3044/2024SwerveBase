package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class XModeCommand extends Command {
    private DriveSubsystem m_driveSubsystem;

    public XModeCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.setX();
    }
}
