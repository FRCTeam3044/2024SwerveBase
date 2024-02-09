package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorManualControlCommand  extends Command {
    private final ElevatorSubsystem m_elevator;
    private final RobotContainer m_robotContainer;

    public ElevatorManualControlCommand(ElevatorSubsystem elevator, RobotContainer robotContainer) {
        m_elevator = elevator;
        m_robotContainer = robotContainer;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        int pov = m_robotContainer.m_driverController.getPOV();
    }
}
