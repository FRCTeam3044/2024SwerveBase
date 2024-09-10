package frc.robot.statemachine.reusable;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.BTrigger;

/**
 * A state in a state machine.
 */
public abstract class State {
    public final Enum<?> name;

    private final StateMachine stateMachine;
    private final ArrayList<BTrigger> triggers = new ArrayList<>();
    private State parentState;
    protected final EventLoop loop = new EventLoop();

    /**
     * Create a new state under the given state machine with a given identifier.
     * 
     * @param stateMachine The state machine this state belongs to
     * @param name         The identifier of this state
     */
    public State(StateMachine stateMachine, Enum<?> name) {
        this.stateMachine = stateMachine;
        this.name = name;
    }

    /**
     * Add a transition to a different state on a trigger.
     * 
     * @param trigger The trigger to transition on
     * @param name    The state to transition to
     * @return This state
     */
    protected State addTransitionTo(Enum<?> name, BTrigger trigger) {
        BTrigger eventTrigger = new BTrigger(loop, trigger);
        State state = stateMachine.states.get(name);
        if (state == null) {
            throw new IllegalStateException("State " + name + " does not exist");
        }
        eventTrigger.onTrue(Commands.run(() -> {
            stateMachine.transitionToState(state);
        }));
        return this;
    }

    /**
     * Set the parent state of this state.
     * 
     * @param state The parent state
     * @return This state
     */
    public State substateOf(Enum<?> state) {
        State tmp = stateMachine.states.get(state);
        if (tmp == null) {
            throw new IllegalStateException("State " + state + " does not exist");
        }
        parentState = tmp;
        return this;
    }

    /**
     * Check if this state is the same as the given state or if it is a substate of
     * the given state.
     * 
     * @param state The state to check against
     * @return True if this state is the same as the given state or if it is a
     *         substate of the given state.
     */
    public boolean is(Enum<?> state) {
        return name == state || (parentState != null && parentState.is(state));
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
     * A trigger that is true when the state machine should be controlling the
     * robot.
     * 
     * @return The trigger
     */
    public BTrigger runningTrg() {
        return new BTrigger(loop, () -> stateMachine.isRunning);
    }

    /**
     * Configure the triggers on this state.
     */
    public abstract void configure();

    /**
     * On enter state
     */
    public void onEnter() {
    }
}
