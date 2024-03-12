package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.State;
import me.nabdev.pathfinding.autos.AutoBoolean;

public class IsState implements AutoBoolean {
    private final StateMachine stateMachine;
    private final State state;

    public IsState(StateMachine stateMachine, String state) {
        this.stateMachine = stateMachine;
        this.state = State.valueOf(state);
    }

    @Override
    public BooleanSupplier getSupplier(BooleanSupplier... children) {
        return this::isState;
    }

    private boolean isState() {
        return stateMachine.getState() == state;
    }

}
