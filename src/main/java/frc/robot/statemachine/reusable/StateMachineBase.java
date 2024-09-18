package frc.robot.statemachine.reusable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class StateMachineBase {
    public State currentState;

    /**
     * Transition to a new state
     * 
     * @param state The state to transition to
     */
    public void transitionTo(State state) {
        State targetState = state.evaluateEntranceState();
        if (currentState != null)
            currentState.onExit();
        currentState = targetState;
        currentState.onEnter();
    }

    /**
     * Execute the state machine
     */
    public void periodic() {
        if (currentState == null) {
            DriverStation.reportWarning("The state machine has not been given an initial state, so it is useless!",
                    null);
            return;
        }
        currentState.checkTransitions();
        currentState.run();
        SmartDashboard.putString("State", currentState.getName());
    }
}
