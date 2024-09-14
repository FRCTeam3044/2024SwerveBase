package frc.robot.statemachine.reusable;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.json.JSONObject;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.BTrigger;

/**
 * A state in a state machine.
 */
public abstract class State {
    public record ChildSelectionInfo(State state, BooleanSupplier condition, int priority) {
    };

    public State parentState;
    public final ArrayList<ChildSelectionInfo> children = new ArrayList<>();

    protected final EventLoop loop = new EventLoop();
    protected final JSONObject parameters;

    private final StateMachine stateMachine;
    private final ArrayList<BTrigger> triggers = new ArrayList<>();
    private boolean hasDefaultChild = false;

    /**
     * Create a new state under the given state machine.
     * 
     * @param stateMachine The state machine this state belongs to
     */
    public State(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
        this.parameters = new JSONObject();
    }

    /**
     * Create a new state under the given state machine.
     * 
     * @param stateMachine The state machine this state belongs to
     */
    public State(StateMachine stateMachine, JSONObject parameters) {
        this.stateMachine = stateMachine;
        this.parameters = parameters;
    }

    /**
     * Add a transition to a different state on a trigger.
     * 
     * @param state   The state to transition to
     * @param trigger The trigger to transition on (event loop will be overwritten)
     * @return This state
     */
    protected State addTransition(State state, Trigger trigger) {
        BTrigger eventTrigger = new BTrigger(loop, trigger);
        eventTrigger.onTrue(Commands.run(() -> {
            stateMachine.transitionTo(state);
        }));
        return this;
    }

    public void onExit() {
        for (BTrigger trigger : triggers) {
            trigger.stopCommands();
        }
    }

    public void run() {
        loop.poll();
    }

    /**
     * On enter state
     */
    public void onEnter() {
    };

    private void addChild(State child, BooleanSupplier condition, int priority, boolean isDefault) {
        if (isDefault) {
            if (hasDefaultChild)
                throw new RuntimeException("A state can only have one default child");
            hasDefaultChild = true;
        }
        children.add(new ChildSelectionInfo(child, condition, priority));
    }

    State evaluateEntranceState() {
        if (children.isEmpty())
            return this;
        ChildSelectionInfo best = null;
        for (ChildSelectionInfo i : children) {
            if ((best == null || best.priority > i.priority) && i.condition.getAsBoolean()) {
                best = i;
            }
        }
        if (best == null) {
            throw new RuntimeException(
                    "A state was unable to determine which child to transition to. Consider adding a default state.");
        }
        return best.state.evaluateEntranceState();
    }

    /**
     * Add a child state to this state.
     * 
     * @param child     The child state
     * @param condition The condition to transition to this state
     * @param priority  The priority of this state
     */
    public void addChild(State child, BooleanSupplier condition, int priority) {
        addChild(child, condition, priority, false);
    }

    /**
     * Add a default child state to this state.
     * 
     * @param child The child state
     */
    public void setDefaultChild(State child) {
        addChild(child, () -> true, Integer.MAX_VALUE, true);
    }
}
