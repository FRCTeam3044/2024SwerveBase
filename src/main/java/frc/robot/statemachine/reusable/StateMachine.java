package frc.robot.statemachine.reusable;

public class StateMachine {
    public State currentState;

    public void transitionTo(State state) {
        State targetState = state.evaluateEntranceState();
        currentState.onExit();
        currentState = targetState;
        currentState.onEnter();
    }

    public void periodic() {
        currentState.run();
    }
}
