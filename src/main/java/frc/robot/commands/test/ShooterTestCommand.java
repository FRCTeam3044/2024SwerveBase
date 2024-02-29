package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTestCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final XboxController m_controller;

    public ShooterTestCommand(ShooterSubsystem shooter, XboxController controller) {
        m_shooter = shooter;
        m_controller = controller;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest();
    }

    @Override
    public void execute() {
        double pov = m_controller.getPOV();
        if (pov == 0) {
            m_shooter.consumeShooterInput(true, false);
        } else if (pov == 180) {
            m_shooter.consumeShooterInput(true, true);
        } else {
            m_shooter.consumeShooterInput(false, false);
        }
    }
}
