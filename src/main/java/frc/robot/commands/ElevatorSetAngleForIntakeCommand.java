package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetAngleForIntakeCommand extends Command {
    private final ElevatorSubsystem m_elevator;

    public ElevatorSetAngleForIntakeCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_elevator.setToIntakeMode();
        m_elevator.pidHandler();
    }
}
