package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.utils.ControllerRumble;

public class DriverShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final TransitSubsystem m_transit;
    public Debouncer m_transitLimitDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    private final CommandXboxController m_controller;
    private boolean hasShot = false;

    public DriverShootCommand(ShooterSubsystem shooter, TransitSubsystem transit, CommandXboxController controller) {
        m_shooter = shooter;
        m_transit = transit;
        m_controller = controller;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        hasShot = false;
    }

    @Override
    public void execute() {
        m_shooter.speakerSpeed();
        m_shooter.handlePID();
        if (m_controller.getRightTriggerAxis() > 0.5 && !hasShot) {
            hasShot = true;
            ControllerRumble.driverSmallShort();
            ControllerRumble.opSmallShort();
            m_shooter.saveShotData();
        }
        if (hasShot) {
            if (m_shooter.shooterAtSpeed()) {
                m_transit.runTransit();
            }
        }

    }

    @Override
    public boolean isFinished() {
        return !m_transitLimitDebouncer.calculate(m_transit.readTransitLimitSwitch()) && hasShot;
    }

}
