package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRunUntilSwitch extends Command {
    private final IntakeSubsystem m_intake;

    public IntakeRunUntilSwitch(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.runIntake();
    }

    /*
     * If true imediately flip limit switch output
     */
    @Override
    public boolean isFinished() {
        return m_intake.readIntakeLimitSwitch();
    }
}
