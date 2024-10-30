package frc.robot.statemachine.states.smart;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.AutoBuilder;
import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class PickupNoteAuto extends State {
    private final Translation2d targetNote;

    public PickupNoteAuto(StateMachineBase stateMachine, JSONObject parameters) {
        super(stateMachine, parameters);
        targetNote = JSONUtils.getTranslation2d(parameters.getJSONObject("notePos"));
        onEnterTrg()
                .onTrue(Commands.deferredProxy(() -> StateCommands.pickupNote(AutoBuilder.allianceFlip(targetNote))));
    }

}
