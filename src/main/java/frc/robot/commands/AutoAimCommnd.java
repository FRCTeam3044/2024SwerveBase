package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.AutoTargetUtils;

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
        Translation2d target = AutoTargetUtils.getShootingTarget().getTranslation();
        double distance = m_drive.getPose().getTranslation().getDistance(target);
        double angle = RobotContainer.m_autoAiming.getAngle(distance);
        m_elevator.setAngle(angle);
    }
}
