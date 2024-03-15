package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ClimberCommand extends Command {
    private final ClimberSubsystem m_climber;
    private final XboxController m_controller;
    private ConfigurableParameter<Double> climberControlDeadband = new ConfigurableParameter<Double>(0.5,
            "Climber Control Deadband");

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
        // boolean isLeftBumperPressed = m_controller.getLeftBumper();
        // boolean isRightBumperPressed = m_controller.getRightBumper();

        // double rightPow = isRightBumperPressed ?
        // ClimberConstants.kClimberManualSpeed.get() : 0;
        // double leftPow = isLeftBumperPressed ?
        // ClimberConstants.kClimberManualSpeed.get() : 0;
        double rightStick = -MathUtil.applyDeadband(m_controller.getRightY(),
                climberControlDeadband.get());
        double leftStick = -MathUtil.applyDeadband(m_controller.getLeftY(),
                climberControlDeadband.get());

        m_climber.consumeClimberInput(leftStick *
                ClimberConstants.kClimberManualSpeed.get(),
                rightStick * ClimberConstants.kClimberManualSpeed.get());
    }
}