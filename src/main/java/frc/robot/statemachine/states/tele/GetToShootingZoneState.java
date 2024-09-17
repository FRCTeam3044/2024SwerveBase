package frc.robot.statemachine.states.tele;

import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class GetToShootingZoneState extends State {
    private Triggers triggers = new Triggers(loop);

    public GetToShootingZoneState(StateMachineBase stateMachine) {
        super(stateMachine);
        active().whileTrue(StateCommands.driveToShootingZone());

        triggers.hasNoteTrg().and(triggers.nearLocationTrg(() -> {
            return AutoTargetUtils.getShootingTarget().getTranslation();
        }, ShooterConstants.kShooterSpinupRange.get()))
                .whileTrue(RobotContainer.shooter.speaker());
    }
}