package frc.robot.statemachine.reusable;

import java.util.function.BooleanSupplier;

import org.json.JSONObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class AutoState extends State {
    protected boolean finished;
    protected JSONObject parameters;

    public AutoState(StateMachine stateMachine, Enum<?> name, JSONObject parameters) {
        super(stateMachine, name);
        this.parameters = parameters;
    }

    protected Command endCmd() {
        return Commands.run(() -> finished = true);
    }

    protected Command endAfterCmd(double time) {
        return Commands.sequence(Commands.waitSeconds(time), endCmd());
    }

    protected Command endIfCmd(BooleanSupplier condition) {
        return Commands.run(() -> {
            if (condition.getAsBoolean()) {
                finished = true;
            }
        });
    }
}
