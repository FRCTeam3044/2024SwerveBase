package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTestCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final XboxController m_controller;

    public IntakeTestCommand(IntakeSubsystem intake, XboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        boolean isBButtonPressed = m_controller.getBButton();
        m_intake.consumeIntakeInput(isBButtonPressed, false);
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }
}
