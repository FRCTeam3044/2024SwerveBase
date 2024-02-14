package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private double targetAngle;

    public ElevatorCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        //
    }

    public void setElevatorTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }
}
