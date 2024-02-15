package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpeakerShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public SpeakerShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.speakerSpeed();
        m_shooter.handlePID();
    }
}
