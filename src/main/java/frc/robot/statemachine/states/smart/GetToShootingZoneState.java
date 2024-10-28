package frc.robot.statemachine.states.smart;

import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.statemachine.StateCommands;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class GetToShootingZoneState extends State {

    public GetToShootingZoneState(StateMachineBase stateMachine) {
        super(stateMachine);
        onEnterTrg().onTrue(StateCommands.driveToShootingZone());

        t(RobotContainer.intake::hasNote).and(Triggers.nearLocation(() -> {
            return AutoTargetUtils.getShootingTarget().getTranslation();
        }, ShooterConstants.kShooterSpinupRange.get()))
                .whileTrue(RobotContainer.shooter.shootPercentage(0.8));
    }
}