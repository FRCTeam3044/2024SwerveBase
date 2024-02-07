package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRunMotorsCommand extends Command {
    private final IntakeSubsystem m_intake;

    public IntakeRunMotorsCommand(IntakeSubsystem intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return m_intake.readIntakeLimitSwitch();
    }
}
