package frc.robot.statemachine.states.smart;

import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class GetToSourceState extends State {
    public GetToSourceState(StateMachineBase stateMachine) {
        super(stateMachine);
        onEnterTrg().onTrue(StateCommands.goToSource());
    }
}