package frc.robot.statemachine.reusable;

import java.util.HashMap;

public class StateMachine {
    public final HashMap<Enum<?>, State> states = new HashMap<>();
    public State currentState;
    protected boolean isRunning = false;

    public State configureState(State state) {
        states.put(state.name, state);
        state.configureTransitions();
        state.configureCommands();
        return state;
    }

    public void transitionToState(State state) {
        currentState.cancel();
        currentState = state;
        currentState.schedule();
    }

    public boolean is(Enum<?> state) {
        return currentState.is(state);
    }

    public void start() {
        currentState.schedule();
    }
}
