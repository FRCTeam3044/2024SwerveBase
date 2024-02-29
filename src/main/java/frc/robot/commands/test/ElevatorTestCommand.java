package frc.robot.commands.test;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorTestCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final XboxController m_controller;

    public ElevatorTestCommand(ElevatorSubsystem elevator, XboxController controller) {
        m_elevator = elevator;
        m_controller = controller;
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }

    @Override
    public void execute() {
        double leftTrigger = m_controller.getLeftTriggerAxis();
        double rightTrigger = m_controller.getRightTriggerAxis();
        leftTrigger = MathUtil.applyDeadband(leftTrigger, 0.4) * ElevatorConstants.kElevatorManualSpeed.get();
        rightTrigger = MathUtil.applyDeadband(rightTrigger, 0.4) * ElevatorConstants.kElevatorManualSpeed.get();
        if (leftTrigger > 0 && rightTrigger > 0) {
            return;
        }
        if (leftTrigger > 0) {
            m_elevator.consumeElevatorInput(-leftTrigger);
        } else if (rightTrigger > 0) {
            m_elevator.consumeElevatorInput(rightTrigger);
        }
    }
}