package frc.robot.statemachine.reusable;

public abstract class StateMachineBase {
    public State currentState;

    /**
     * Transition to a new state
     * 
     * @param state The state to transition to
     */
    public void transitionTo(State state) {
        State targetState = state.evaluateEntranceState();
        currentState.onExit();
        currentState = targetState;
        currentState.onEnter();
    }

    /**
     * Execute the state machine
     */
    public void periodic() {
        currentState.checkTransitions();
        currentState.run();
        System.out.println(currentState.getName());
    }
}
