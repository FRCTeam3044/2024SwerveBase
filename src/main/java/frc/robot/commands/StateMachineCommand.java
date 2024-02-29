package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;

public class StateMachineCommand extends Command {
    private StateMachine m_stateMachine;
    private Command stateCommand;

    public StateMachineCommand(StateMachine stateMachine) {
        m_stateMachine = stateMachine;
    }

    @Override
    public void initialize() {
        stateCommand = m_stateMachine.getDesiredCommand();
        if (stateCommand != null) {
            stateCommand.schedule();
        }
    }

    @Override
    public void execute() {
        if (!m_stateMachine.changedDesiredCommand)
            return;
        if (stateCommand != null) {
            stateCommand.cancel();
        }
        stateCommand = m_stateMachine.getDesiredCommand();
        if (stateCommand != null) {
            stateCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (stateCommand != null) {
            stateCommand.cancel();
        }
    }
}
