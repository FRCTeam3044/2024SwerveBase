package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.StateMachine.State;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ShootIfReady extends Command {
    private TransitSubsystem m_transit;
    private DriveSubsystem m_drive;
    private ConfigurableParameter<Double> m_shooterSpinupRange = new ConfigurableParameter<Double>(7.5,
            "Shooter Spinup Range");

    public ShootIfReady(TransitSubsystem transit, DriveSubsystem drive) {
        m_drive = drive;
        m_transit = transit;
        addRequirements(m_transit);
    }

    @Override
    public void execute() {
        if(RobotContainer.stateMachine.getState() === State.READY_TO_SHOOT) {
            m_transit.shoot();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
