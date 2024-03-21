package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class AmpShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public AmpShooterCommand(ShooterSubsystem shooter, TransitSubsystem transit) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.ampSpeed();
        m_shooter.handlePID();
        // m_shooter.consumeShooterInput(true, false);
        // if(m_shooter.shooterAtSpeed()){
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.consumeShooterInput(false, false);
    }
}
