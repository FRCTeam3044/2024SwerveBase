package frc.robot.statemachine.states;

import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachine;

public class RobotState extends State {
    Triggers triggers = new Triggers(loop);

    public RobotState(StateMachine stateMachine) {
        super(stateMachine);
    }
}
