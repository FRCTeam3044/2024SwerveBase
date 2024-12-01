package frc.robot.statemachine.reusable;

import java.util.Stack;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.statemachine.reusable.State.TransitionInfo;

public abstract class StateMachineBase {
    /**
     * The root state of the state tree. This state will always be active, and all
     * other states will be children of this state.
     */
    public State rootState = new State(this) {
        {
            name = "Root";
        }
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
        SmartDashboard.putString("State Tree", getObjectForState(rootState).toString());
    }

    public void registerToRootState(State... state) {
        for (State s : state) {
            rootState.children.add(s);
            s.setParentState(rootState);
        }
    }

    private void checkTransitions() {
        State newState = traverseTransitions(currentState);
        if (newState != currentState) {
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

    JSONObject getObjectForState(State state) {
        JSONObject obj = new JSONObject();
        obj.put("name", state.getName());
        JSONArray children = new JSONArray();
        for (State child : state.children) {
            children.put(getObjectForState(child));
        }
        obj.put("children", children);
        if (state.parentState == null)
            return obj;
        JSONArray transitions = new JSONArray();
        for (TransitionInfo transition : state.parentState.transitions.get(state)) {
            JSONObject transitionObj = new JSONObject();
            transitionObj.put("name", transition.name());
            transitionObj.put("target", transition.state().getDeepName());
            transitions.put(transitionObj);
        }
        obj.put("transitions", transitions);
        return obj;

    }
}
