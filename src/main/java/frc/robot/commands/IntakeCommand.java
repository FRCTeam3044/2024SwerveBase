package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final XboxController m_controller;

    public IntakeCommand(IntakeSubsystem intake, XboxController controller) {
        m_intake = intake;
        m_controller = controller;
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
        boolean isBButtonPressed = m_controller.getBButtonPressed();

        m_intake.consumeIntakeInput(isBButtonPressed);
    }
}
