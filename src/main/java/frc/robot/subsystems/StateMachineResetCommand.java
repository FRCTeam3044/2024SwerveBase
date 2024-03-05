package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class StateMachineResetCommand extends Command {
    private final StateMachine stateMachine;

    public StateMachineResetCommand(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
    public void initialize() {
        stateMachine.reset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
