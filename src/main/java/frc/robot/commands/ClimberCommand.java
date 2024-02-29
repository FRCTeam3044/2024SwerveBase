package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    private final ClimberSubsystem m_climber;
    private final XboxController m_controller;

    public ClimberCommand(ClimberSubsystem climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;
        addRequirements(m_climber);
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
        boolean isLeftBumperPressed = m_controller.getLeftBumper();
        boolean isRightBumperPressed = m_controller.getRightBumper();

        double rightPow = isRightBumperPressed ? ClimberConstants.kClimberManualSpeed.get() : 0;
        double leftPow = isLeftBumperPressed ? ClimberConstants.kClimberManualSpeed.get() : 0;
        m_climber.consumeClimberInput(rightPow, leftPow);
    }
}