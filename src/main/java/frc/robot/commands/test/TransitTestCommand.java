package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitSubsystem;

public class TransitTestCommand extends Command {
    private final TransitSubsystem m_transit;
    private final XboxController m_controller;

    public TransitTestCommand(TransitSubsystem transit, XboxController controller) {
        m_transit = transit;
        m_controller = controller;
        addRequirements(m_transit);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        boolean isXButtonPressed = m_controller.getXButtonPressed();
        m_transit.consumeTransitInput(isXButtonPressed);
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }
}
