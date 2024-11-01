package frc.robot.statemachine.states.smart;

import frc.robot.RobotContainer;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class ShootState extends State {
    public ShootState(StateMachineBase stateMachine) {
        super(stateMachine);
        onEnterTrg().onTrue(RobotContainer.m_robotDrive.trackPoint(AutoTargetUtils::getShootingTarget,
                true));

        onEnterTrg().onTrue(RobotContainer.shooter.speaker());

        t(Triggers.readyToShoot()).onTrue(RobotContainer.transit.run());
    }
}