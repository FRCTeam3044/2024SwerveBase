package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoAimCommand extends Command {
    private ElevatorSubsystem m_elevator;
    private DriveSubsystem m_drive;

    public AutoAimCommand(ElevatorSubsystem elevator, DriveSubsystem drive) {
        long startTime = System.currentTimeMillis();
        m_drive = drive;
        m_elevator = elevator;
        addRequirements(m_elevator);
        // System.out.println(getName() + " Took " + (double)
        // (System.currentTimeMillis() - startTime) / 1000);
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
