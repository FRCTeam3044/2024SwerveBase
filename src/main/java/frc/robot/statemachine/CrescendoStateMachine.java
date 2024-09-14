package frc.robot.statemachine;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachine;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.DriverAssistState;
import frc.robot.statemachine.states.tele.ManualState;

public class CrescendoStateMachine extends StateMachine {
    public static CrescendoStateMachine INSTANCE = new CrescendoStateMachine();

    public CrescendoStateMachine() {
        super();
        if (INSTANCE != null) {
            throw new IllegalStateException("Cannot create another instance of singleton class");
        }
        State teleop = new TeleState(this);
        State auto = new AutoState(this);
        State test = new TestState(this);
        State disabled = new DisabledState(this);

        teleop.setDefaultChild(new ManualState(this));
        teleop.addChild(new DriverAssistState(this), null, 0);
    }
}
