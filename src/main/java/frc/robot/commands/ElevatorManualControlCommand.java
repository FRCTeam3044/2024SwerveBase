package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorManualControlCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final XboxController m_controller;

    public ElevatorManualControlCommand(ElevatorSubsystem elevator, XboxController controller) {
        m_elevator = elevator;
        m_controller = controller;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double leftStickY = m_controller.getLeftY();

        leftStickY = leftStickY * ElevatorConstants.kElevatorManualSpeed.get();
        m_elevator.consumeElevatorInput(leftStickY);
    }
}
