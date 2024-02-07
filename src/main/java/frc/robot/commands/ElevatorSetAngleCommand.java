package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetAngleCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final XboxController m_controller;

    public ElevatorSetAngleCommand(ElevatorSubsystem elevator, XboxController controller) {
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
        m_controller.
        m_elevator.
    }
}
