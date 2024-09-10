package frc.robot.statemachine.autostates;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.statemachine.AutoCommands;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.AutoState;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.StateMachine;

public class PickupNoteState extends AutoState {
    private Triggers triggers = new Triggers(this.loop);

    // Load parameters
    Translation2d notePos = JSONUtils.getTranslation2d(parameters.getJSONObject("notePos"));
    Pose2d notePose = new Pose2d(notePos, new Rotation2d());

    public PickupNoteState(StateMachine stateMachine, Enum<?> name, JSONObject parameters) {
        super(stateMachine, name, parameters);
    }

    @Override
    public void configure() {
        runningTrg().onTrue(endIfCmd(() -> !triggers.noteDetectedNearTrg(notePos).getAsBoolean()));

        triggers.noteDetectedNearTrg(notePos)
                .onTrue(AutoCommands.pickupNoteAt(notePos).onlyWhile(triggers.hasNoteTrg().negate())
                        .withName("Auto Pickup Note At"));

        triggers.hasNoteTrg().onTrue(endCmd());
    }
}
