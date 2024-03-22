package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.StateMachine.State;

public class ShootIfReady extends Command {
    private TransitSubsystem m_transit;

    public ShootIfReady(TransitSubsystem transit) {
        m_transit = transit;
        addRequirements(m_transit);
    }

    @Override
    public void execute() {
        if (RobotContainer.stateMachine.getState() == State.READY_TO_SHOOT) {
            m_transit.runTransit();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_transit.stopTransit();
    }

}
