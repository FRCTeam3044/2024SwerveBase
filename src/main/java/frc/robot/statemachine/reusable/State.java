package frc.robot.statemachine.reusable;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.BTrigger;

/**
 * A state in a state machine.
 */
public abstract class State {
    public record ChildSelectionInfo(State state, BooleanSupplier condition, int priority) {
    };

    public State parentState;
    public final ArrayList<ChildSelectionInfo> children = new ArrayList<>();

    private final StateMachine stateMachine;
    private final ArrayList<BTrigger> triggers = new ArrayList<>();
    protected final EventLoop loop = new EventLoop();

    /**
     * Create a new state under the given state machine.
     * 
     * @param stateMachine The state machine this state belongs to
     */
    public State(StateMachine stateMachine, JSONObject parameters) {
        this.stateMachine = stateMachine;
    }

    /**
     * Add a transition to a different state on a trigger.
     * 
     * @param state   The state to transition to
     * @param trigger The trigger to transition on (event loop will be overwritten)
     * @return This state
     */
    protected State addTransition(State state, BTrigger trigger) {
        BTrigger eventTrigger = new BTrigger(loop, trigger);
        eventTrigger.onTrue(Commands.run(() -> {
            stateMachine.transitionToState(state);
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
    public abstract void onEnter();

    public State evaluateEntranceState(){
        if(children.isEmpty()) return this;
        ChildSelectionInfo best;
        for(ChildSelectionInfo i : children){
            if((best == null || best.priority > i.priority) && i.condition.getAsBoolean()){
                best = i;
            }
        }
        if(best == null){
            throw new RuntimeException("A state was unable to determine which child to transition to. Consider adding a default state.");
        }
        return best.evaluateEntranceState();
    }
}
