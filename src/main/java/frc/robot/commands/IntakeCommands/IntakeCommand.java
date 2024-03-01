package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final XboxController m_controller;

    public IntakeCommand(IntakeSubsystem intake, XboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TODO: Which button?
        boolean isBButtonPressed = m_controller.getBButton();

        m_intake.consumeIntakeInput(isBButtonPressed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
