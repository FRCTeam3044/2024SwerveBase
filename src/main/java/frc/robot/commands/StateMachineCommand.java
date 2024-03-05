package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.utils.ControllerRumble;

public class StateMachineCommand extends Command {
    private StateMachine m_stateMachine;
    private Command stateCommand;
    private State lastState;

    public StateMachineCommand(StateMachine stateMachine) {
        m_stateMachine = stateMachine;
    }

    @Override
    public void initialize() {
        stateCommand = m_stateMachine.getDesiredCommand();
        if (stateCommand != null) {
            stateCommand.schedule();
        }
        m_stateMachine.lostNote = false;
        lastState = m_stateMachine.getState();
        if (lastState == State.TARGETING_NOTE) {
            ControllerRumble.driverbigShort();
        }
    }

    @Override
    public void execute() {
        if (m_stateMachine.lostNote) {
            ControllerRumble.driverPulseLong();
            m_stateMachine.lostNote = false;
        }
        if (!m_stateMachine.changedDesiredCommand)
            return;
        if (stateCommand != null) {
            stateCommand.cancel();
        }
        stateCommand = m_stateMachine.getDesiredCommand();
        if (stateCommand != null) {
            stateCommand.schedule();
        }
        if (m_stateMachine.getState() != lastState) {
            lastState = m_stateMachine.getState();
            if (lastState == State.TARGETING_NOTE) {
                ControllerRumble.driverbigShort();
            }
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
