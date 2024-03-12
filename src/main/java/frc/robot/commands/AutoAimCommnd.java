package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoAimCommnd extends Command {
    private ElevatorSubsystem m_elevator;
    private DriveSubsystem m_drive;

    public AutoAimCommnd(ElevatorSubsystem elevator, DriveSubsystem drive) {
        m_drive = drive;
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        double angle = RobotContainer.m_autoAiming.getAngle(m_drive.distanceToShootingTarget);
        m_elevator.setAngle(angle);
        m_elevator.pidHandler();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
