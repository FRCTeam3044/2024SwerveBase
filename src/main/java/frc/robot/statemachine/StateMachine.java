package frc.robot.statemachine;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.DriverAssistState;
import frc.robot.statemachine.states.tele.ManualState;

public class StateMachine extends StateMachineBase {
    public static StateMachine INSTANCE = new StateMachine();

    // Robot Control states
    State teleop = new TeleState(this);
    State auto = new AutoState(this);
    State test = new TestState(this);
    State disabled = new DisabledState(this);

    public StateMachine() {
        super();
        if (INSTANCE != null) {
            throw new IllegalStateException("Cannot create another instance of singleton class");
        }

        currentState = disabled;

        teleop.withModeTransitions(disabled, teleop, auto, test)
                .withDefaultChild(new ManualState(this))
                .withChild(new DriverAssistState(this), () -> false, 0);

        auto.withModeTransitions(disabled, teleop, auto, test);
        test.withModeTransitions(disabled, teleop, auto, test);
        disabled.withModeTransitions(disabled, teleop, auto, test);
    }
}
