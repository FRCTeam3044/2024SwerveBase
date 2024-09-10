package frc.robot.statemachine.states;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.CrescendoStateMachine.States;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachine;
import frc.robot.utils.BTrigger;

public class NoNoteState extends State {
    public NoNoteState(StateMachine stateMachine, Enum<?> name) {
        super(stateMachine, name);
    }

    @Override
    public void configureTransitions() {
        // Configure all transitions from this state here
        addTransitionTo(States.HAS_NOTE,
                new BTrigger(this.loop, () -> RobotContainer.intake
                        .getCurrent() < StateMachineConstants.kIntakeCurrentThreshold.get()));
        BooleanSupplier toTargetingNote = () -> {
            if (StateMachineConstants.kHasNoteDebouncer.calculate(RobotContainer.m_noteDetection.hasNote)) {
                double distance = RobotContainer.m_noteDetection.getClosestNoteDistance();
                return (distance < StateMachineConstants.kNoteDetectionDistance.get());
            }
            return false;
        };
        addTransitionTo(States.TARGETING_NOTE, new BTrigger(this.loop, toTargetingNote));
    }

    @Override
    public void configureCommands() {
        runningTrg().whileTrue(StateCommands.goToSource());
    }

}
