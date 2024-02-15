package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public double ampAngle;

    public AmpShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        ampAngle = m_shooter.speakerRPM;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.topSpeakerPidHandler(ampAngle);
        m_shooter.bottomSpeakerPidHandler(-ampAngle);
    }
}
