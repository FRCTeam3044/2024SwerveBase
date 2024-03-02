package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSlowCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public ShooterSlowCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.consumeShooterInput(true, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.consumeShooterInput(false, false);
    }
}
