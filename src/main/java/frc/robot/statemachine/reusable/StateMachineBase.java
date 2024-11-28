package frc.robot.statemachine.reusable;

import java.util.Stack;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class StateMachineBase {
    /**
     * The root state of the state tree. This state will always be active, and all
     * other states will be children of this state.
     */
    public State rootState = new State(this) {
    };

    /**
     * The current active leaf state of the state tree.
     */
    public State currentState;

    /**
     * Execute the state machine
     */
    public void periodic() {
        if (currentState == null) {
            DriverStation.reportWarning("The state machine has not been given an initial state, so it is useless!",
                    null);
            return;
        }
        checkTransitions();
        currentState.run();
        SmartDashboard.putString("State", currentState.getDeepName());
    }

    private void checkTransitions() {
        State newState = traverseTransitions(currentState);
        if (newState != currentState) {
            System.out.println("Transitioning from " + currentState.getDeepName() + " to " + newState.getDeepName());
            Stack<State> before = getStateTree(currentState);
            Stack<State> after = getStateTree(newState);

            while (!before.isEmpty() && !after.isEmpty() && before.peek() == after.peek()) {
                before.pop();
                after.pop();
            }

            while (!before.isEmpty()) {
                before.pop().onExit();
            }

            while (!after.isEmpty()) {
                after.pop().onEnter();
            }

            currentState = newState;
        }
    }

    State traverseTransitions(State state) {
        return rootState.evalTransitions(getStateTree(state));
    }

    Stack<State> getStateTree(State state) {
        Stack<State> stateTree = new Stack<>();
        State cur = state;
        while (cur.parentState != null) {
            stateTree.push(cur);
            cur = cur.parentState;
        }
        return stateTree;
    }
}
