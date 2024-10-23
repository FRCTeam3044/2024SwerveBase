package frc.robot.statemachine.states.smart;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.SmartTrigger;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class PickupNoteAuto extends State {
    private final Translation2d targetNote;

    public PickupNoteAuto(StateMachineBase stateMachine, JSONObject parameters) {
        super(stateMachine, parameters);
        targetNote = JSONUtils.getTranslation2d(parameters.getJSONObject("notePos"));
        // SmartTrigger nearNote = t(Triggers.nearLocation(targetNote, 0.3));
        // onEnterTrg().and(nearNote.negate())
        // .onTrue(RobotContainer.m_robotDrive.goToAndTrackPoint(targetNote, targetNote,
        // true, false));
        // nearNote.whileTrue((StateCommands.pickupNote(targetNote)));
        onEnterTrg().onTrue(StateCommands.pickupNote(targetNote));
    }

}
