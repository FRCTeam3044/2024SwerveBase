package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AmpShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final TransitSubsystem m_transit;
    private final Timer m_timer = new Timer();
    private final ConfigurableParameter<Double> m_ampShooterSpinupTime = new ConfigurableParameter<Double>(
            2.0, "Amp Shooter Spinup Time");
    private boolean ranTransit = false;

    public AmpShooterCommand(ShooterSubsystem shooter, TransitSubsystem transit) {
        m_shooter = shooter;
        m_transit = transit;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        ranTransit = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_shooter.ampSpeed();
        m_shooter.handlePID(true);
        // m_shooter.consumeShooterInput(true, false);
        // if(m_shooter.shooterAtSpeed()){
        if (m_timer.get() > m_ampShooterSpinupTime.get()) {
            ranTransit = true;
            // m_transit.runTransit();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.consumeShooterInput(false, false);
        if (ranTransit)
            m_transit.stopTransit();
    }
}
