package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetAngleForAmpCommand extends Command {
    private final ElevatorSubsystem m_elevator;

    public ElevatorSetAngleForAmpCommand(ElevatorSubsystem elevator, XboxController controller) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        m_elevator.ampPidHandler();
    }
}
