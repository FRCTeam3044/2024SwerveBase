package frc.robot.statemachine.states.tele;

import frc.robot.RobotContainer;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class ShootState extends State {
    private Triggers triggers = new Triggers(loop);
    public ShootState(StateMachineBase stateMachine) {
        super(stateMachine);
        active().whileTrue(RobotContainer.m_robotDrive.trackPoint(AutoTargetUtils::getShootingTarget,
        true));

        active().whileTrue(RobotContainer.shooter.speaker());

        triggers.readyToShootTrg().onTrue(RobotContainer.transit.run());
    }
}