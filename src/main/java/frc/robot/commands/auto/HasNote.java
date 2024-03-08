package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.State;
import me.nabdev.pathfinding.autos.AutoBoolean;

public class HasNote implements AutoBoolean {
    private final StateMachine stateMachine;

    public HasNote(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
    public BooleanSupplier getSupplier(BooleanSupplier... children) {
        return this::hasNote;
    }

    private boolean hasNote() {
        return (stateMachine.getState() == State.OWNS_NOTE || stateMachine.getState() == State.NOTE_LOADED
                || stateMachine.getState() == State.READY_TO_SHOOT);
    }

}
