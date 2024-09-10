package frc.robot.statemachine.autostates;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.AutoState;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.StateMachine;

public class GetToNoteState extends AutoState {
    private Triggers triggers = new Triggers(this.loop);

    // Load parameters
    Translation2d notePos = JSONUtils.getTranslation2d(parameters.getJSONObject("notePos"));
    Pose2d notePose = new Pose2d(notePos, new Rotation2d());

    public GetToNoteState(StateMachine stateMachine, Enum<?> name, JSONObject parameters) {
        super(stateMachine, name, parameters);
    }

    @Override
    public void configure() {
        runningTrg().whileTrue(RobotContainer.m_robotDrive.goToAndTrackPoint(notePose, notePose, false, false));

        triggers.nearLocationTrg(notePos).and(triggers.noteDetectedNearTrg(notePos)).onTrue(endCmd());

        triggers.nearLocationTrg(notePos).and(triggers.noteDetectedNearTrg(notePos).negate())
                .whileTrue(endAfterCmd(0.2));
    }
}
