package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class IntakeAndThenSecond extends Command {
    private final ConfigurableParameter<Double> intakeRuntime = new ConfigurableParameter<Double>(1.0,
            "Intake Auto Runtime");
    private final IntakeSubsystem m_intake;

    public IntakeAndThenSecond(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.runIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Command next = new IntakeCommand(m_intake);
        next = next.raceWith(new WaitCommand(intakeRuntime.get()));
        next.schedule();
    }
}
