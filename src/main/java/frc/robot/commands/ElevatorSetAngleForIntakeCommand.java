package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetAngleForIntakeCommand extends Command {
    private final ElevatorSubsystem m_elevator;

    public double intakeAngle;

    public ElevatorSetAngleForIntakeCommand(ElevatorSubsystem elevator, XboxController controller) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        intakeAngle = m_elevator.intakeAngle;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_elevator.pidHandler(intakeAngle);
    }
}