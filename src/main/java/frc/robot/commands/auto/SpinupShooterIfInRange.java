package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.oxconfig.ConfigurableParameter;

public class SpinupShooterIfInRange extends Command {
    private ShooterSubsystem m_shooter;
    private DriveSubsystem m_drive;
    private ConfigurableParameter<Double> m_shooterSpinupRange = new ConfigurableParameter<Double>(7.5,
            "Shooter Spinup Range");

    public SpinupShooterIfInRange(ShooterSubsystem shooter, DriveSubsystem drive) {
        m_drive = drive;
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        Translation2d target = AutoTargetUtils.getShootingTarget().getTranslation();
        double distance = m_drive.getPose().getTranslation().getDistance(target);
        if (distance < m_shooterSpinupRange.get()) {
            m_shooter.speakerSpeed();
            // m_shooter.consumeShooterInput(true, false);
        } else {
            m_shooter.stopShooter();
            m_shooter.consumeShooterInput(false, false);
        }
        m_shooter.handlePID();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.consumeShooterInput(false, false);
    }

}
