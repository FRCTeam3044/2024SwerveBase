package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ManualLobCommand extends Command {
    private final ShooterSubsystem m_shooter;
    // private final ConfigurableParameter<Double> m_shooterSpinupTime = new
    // ConfigurableParameter<Double>(
    // 2.0, "Shooter Spinup Time");
    private static final ConfigurableParameter<Double> m_shooterLobSpeed = new ConfigurableParameter<Double>(
            2000.0, "Lob Speed");

    public ManualLobCommand(ShooterSubsystem shooter, TransitSubsystem transit) {
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
        m_shooter.setShooterRPM(m_shooterLobSpeed.get());
        m_shooter.handlePID();
        // m_shooter.consumeShooterInput(true, false);
        // if(m_shooter.shooterAtSpeed()){
        // if (m_timer.get() > m_shooterSpinupTime.get()) {
        // ranTransit = true;
        // // m_transit.runTransit();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.consumeShooterInput(false, false);
    }
}
