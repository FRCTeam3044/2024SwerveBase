package frc.robot.statemachine.states.smart;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Commands;
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
        onEnterTrg().onTrue(Commands.deadline(Commands.waitSeconds(1.5), RobotContainer.intake.run()));
        onEnterTrg().onTrue(RobotContainer.elevator.autoAim(RobotContainer.m_robotDrive));

        t(RobotContainer.intake::hasNote).and(Triggers.nearLocation(() -> {
            return AutoTargetUtils.getShootingTarget().getTranslation();
        }, ShooterConstants.kShooterSpinupRange.get()))
                .whileTrue(RobotContainer.shooter.speaker());
    }
}