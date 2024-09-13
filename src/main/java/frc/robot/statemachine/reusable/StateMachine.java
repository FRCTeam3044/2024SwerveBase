package frc.robot.statemachine.reusable;

import java.util.HashMap;

public class StateMachine {
    public State currentState;
    protected boolean isRunning = false;

    public State configureState(State state) {
        state.configure();
        return state;
    }

    public void transitionToState(State state) {
        State targetState = state.evaluateEntranceState();
        currentState.onExit();
        currentState = targetState;
        currentState.onEnter();
    }

    public void periodic() {
        currentState.run();
    }
}
