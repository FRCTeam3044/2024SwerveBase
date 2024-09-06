package frc.robot.statemachine;

import frc.robot.statemachine.reusable.StateMachine;
import frc.robot.statemachine.states.NoNoteState;
import frc.robot.statemachine.states.TargetingState;

public class CrescendoStateMachine extends StateMachine {
    public enum States {
        NO_NOTE,
        TARGETING_NOTE,
        HAS_NOTE,
        READY_TO_SHOOT,
        SHOOTING
    }

    public static CrescendoStateMachine instance;

    public static CrescendoStateMachine getInstance() {
        if (instance == null) {
            instance = new CrescendoStateMachine();
        }
        return instance;
    }

    public CrescendoStateMachine() {
        super();
        if (instance != null) {
            throw new IllegalStateException("Cannot create another instance of singleton class");
        }
        configureState(new NoNoteState(this, States.NO_NOTE));
        configureState(new TargetingState(this, States.TARGETING_NOTE)).substateOf(States.NO_NOTE);
    }
}
