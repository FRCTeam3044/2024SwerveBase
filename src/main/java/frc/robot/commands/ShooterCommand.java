package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final XboxController m_controller;

    public ShooterCommand(ShooterSubsystem shooter, XboxController controller) {
        m_shooter = shooter;
        m_controller = controller;
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
        boolean isAButtonPressed = m_controller.getAButtonPressed();

        m_shooter.consumeShooterInput(isAButtonPressed);
    }
}
