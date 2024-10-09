package frc.robot.statemachine.states.smart;

import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class PickupNoteState extends State {
    public PickupNoteState(StateMachineBase stateMachine) {
        super(stateMachine);
        onEnterTrg().onTrue(StateCommands.pickupNote());
    }
}