package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final RobotContainer m_robotContainer;

    public ElevatorCommand(ElevatorSubsystem elevator, RobotContainer container) {
        m_elevator = elevator;
        m_robotContainer = container;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double getLeftY = m_robotContainer.m_operatorController.getLeftY();
        

        m_elevator.consumeElevatorInput(getLeftY);
    }
}
