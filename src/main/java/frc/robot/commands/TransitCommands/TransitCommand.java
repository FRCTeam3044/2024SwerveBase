package frc.robot.commands.TransitCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitSubsystem;

public class TransitCommand extends Command {
    private final TransitSubsystem m_transit;

    public TransitCommand(TransitSubsystem transit) {
        m_transit = transit;
        addRequirements(m_transit);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_transit.runTransit();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_transit.stopTransit();
    }
}
