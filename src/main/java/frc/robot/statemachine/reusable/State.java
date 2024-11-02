package frc.robot.statemachine.reusable;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A state in a state machine.
 */
public abstract class State {
    public record TransitionInfo(State state, BooleanSupplier condition, int priority) {
    }

    public State parentState;
    public final ArrayList<TransitionInfo> children = new ArrayList<>();

    protected final SmartEventLoop loop = new SmartEventLoop();
    protected final JSONObject parameters;

    private final StateMachineBase stateMachine;
    private final ArrayList<TransitionInfo> transitions = new ArrayList<>();
    private boolean hasDefaultChild = false;
    private String name = this.getClass().getSimpleName();
    protected boolean onEnter = false;

    /**
     * Create a new state under the given state machine.
     * 
     * @param stateMachine The state machine this state belongs to
     */
    public State(StateMachineBase stateMachine) {
        this.stateMachine = stateMachine;
        this.parameters = new JSONObject();
    }

    /**
     * Create a new state under the given state machine.
     * 
     * @param stateMachine The state machine this state belongs to
     */
    public State(StateMachineBase stateMachine, JSONObject parameters) {
        this.stateMachine = stateMachine;
        this.parameters = parameters;
    }

    /**
     * Add a transition to a different state on a condition.
     * 
     * @param state     The state to transition to
     * @param condition The condition to transition on
     * @return This state
     */
    public State withTransition(State state, BooleanSupplier condition) {
        return withTransition(state, condition, Integer.MAX_VALUE);
    }

    /**
     * Add a transition to a different state on a condition.
     * 
     * @param state     The state to transition to
     * @param condition The condition to transition on
     * @param priority  The priority of this transition
     * @return This state
     */
    public State withTransition(State state, BooleanSupplier condition, int priority) {
        return withTransition(new TransitionInfo(state, condition, priority));
    }

    /**
     * Add a transition to a different state on a condition.
     * 
     * @param transition The transition info to add
     * @return This state
     */
    public State withTransition(TransitionInfo transition) {
        transitions.add(transition);
        return this;
    }

    /**
     * Configure mode transitions for this state.
     * 
     * @param disabled The state to transition to when disabled
     * @param teleop   The state to transition to when teleop is enabled
     * @param auto     The state to transition to when auto is enabled
     * @param test     The state to transition to when test is enabled
     */
    public State withModeTransitions(State disabled, State teleop, State auto, State test) {
        if (disabled != this)
            withTransition(disabled, DriverStation::isDisabled);
        if (teleop != this)
            withTransition(teleop, DriverStation::isTeleopEnabled);
        if (auto != this)
            withTransition(auto, DriverStation::isAutonomousEnabled);
        if (test != this)
            withTransition(test, DriverStation::isTestEnabled);
        return this;
    }

    /**
     * Add a child state to this state.
     *
     * @param child     The child state
     * @param condition The condition to transition to this state
     * @param priority  The priority of this state
     * 
     * @return This state
     */
    public State withChild(State child, BooleanSupplier condition, int priority) {
        addChild(child, condition, priority, false);
        return this;
    }

    /**
     * Add a child state to this state (will never be entered by default)
     */
    public State withChild(State child) {
        addChild(child, () -> false, Integer.MAX_VALUE, false);
        return this;
    }

    /**
     * Add a default child state to this state.
     *
     * @param child The child state
     * @return This state
     */
    public State withDefaultChild(State child) {
        addChild(child, () -> true, Integer.MAX_VALUE, true);
        return this;
    }

    /**
     * Remove all children from this state.
     * 
     * @return
     */
    public State withNoChildren() {
        children.clear();
        return this;
    }

    /**
     * Set the name of this state.
     * 
     * @return This state
     */
    public State withName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Get the name of this state.
     * 
     * @return The name of this state
     */
    public String getName() {
        return name;
    }

    /**
     * Get the name of this state, including the names of all parent states.
     * 
     * @return The recurisive name of this state
     */
    public String getDeepName() {
        if (parentState == null)
            return getName();
        return parentState.getDeepName() + "/" + getName();
    }

    /**
     * Get the parameters of this state.
     */
    public boolean is(State state) {
        if (state == this)
            return true;

        if (parentState == null)
            return false;

        return parentState.is(state);
    }

    /**
     * Fires when the state is exited
     */
    public void onExit() {
        loop.stop();
        onEnter = false;
    }

    /**
     * Fires when the state is entered
     */
    public void onEnter() {
        onEnter = true;
        if (parentState != null)
            parentState.onEnter();
    };

    public SmartTrigger t(BooleanSupplier condition) {
        return new SmartTrigger(loop, condition);
    }

    protected SmartTrigger onEnterTrg() {
        return new SmartTrigger(loop, () -> onEnter);
    }

    /**
     * WARNING - activeTrg does not experience a rising edge, so it will not fire!
     * Use only with runWhileTrue and runWhileFalse, or compositions.
     * 
     * @return
     */
    protected SmartTrigger activeTrg() {
        return new SmartTrigger(loop, () -> stateMachine.currentState.is(this));
    }

    void run() {
        if (parentState != null)
            parentState.run();

        loop.poll();
        // Yes, this is intentional. We want to "activate" the state after the first
        // poll. This is done so that the active() triggers actually fire (since
        // otherwise, the event loop would only be polled when active is true, so the
        // trigger would never fire)
        onEnter = false;
    }

    boolean checkTransitions() {
        if (parentState != null) {
            boolean didParentTransition = parentState.checkTransitions();
            if (didParentTransition)
                return true;
        }

        TransitionInfo next = evaluateTransition(transitions);
        if (next != null) {
            stateMachine.transitionTo(next.state);
            return true;
        }
        return false;
    }

    private TransitionInfo evaluateTransition(ArrayList<TransitionInfo> transitions) {
        TransitionInfo best = null;
        for (TransitionInfo i : transitions) {
            if ((best == null || best.priority > i.priority) && i.condition.getAsBoolean()) {
                best = i;
            }
        }
        return best;
    }

    State evaluateEntranceState() {
        if (children.isEmpty())
            return this;
        TransitionInfo next = evaluateTransition(children);
        if (next == null || next.state == null)
            throw new RuntimeException(
                    "A state was unable to determine which child to transition to. Consider adding a default state.");
        return next.state.evaluateEntranceState();
    }

    void setParentState(State parentState) {
        if (this.parentState != null)
            throw new RuntimeException("A state can only have one parent state");
        this.parentState = parentState;
    }

    private void addChild(State child, BooleanSupplier condition, int priority, boolean isDefault) {
        if (isDefault) {
            if (hasDefaultChild)
                throw new RuntimeException("A state can only have one default child");
            hasDefaultChild = true;
        }
        child.setParentState(this);
        children.add(new TransitionInfo(child, condition, priority));
    }
}
