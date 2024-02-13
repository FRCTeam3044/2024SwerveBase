package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public double ampRPM;

    public AmpShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        ampRPM = m_shooter.ampRPM;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.speakerPidHandler(ampRPM);
    }
}
