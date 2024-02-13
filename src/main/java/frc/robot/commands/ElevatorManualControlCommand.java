package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorManualControlCommand  extends Command {
    private final ElevatorSubsystem m_elevator;

    public ElevatorManualControlCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_elevator.consumeElevatorInput();
    }
}
