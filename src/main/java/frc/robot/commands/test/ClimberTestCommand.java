package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestCommand extends Command {
    private final ClimberSubsystem m_climber;
    private final XboxController m_controller;

    public ClimberTestCommand(ClimberSubsystem climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;
        addRequirements(m_climber);
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }

    @Override
    public void execute() {
        boolean isLeftBumperPressed = m_controller.getLeftBumper();
        boolean isRightBumperPressed = m_controller.getRightBumper();
        double rightY = -m_controller.getRightY();

        double rightPow = isRightBumperPressed ? ClimberConstants.kClimberManualSpeed.get() * rightY : 0;
        double leftPow = isLeftBumperPressed ? ClimberConstants.kClimberManualSpeed.get() * rightY : 0;
        m_climber.consumeClimberInput(rightPow, leftPow);
    }
}