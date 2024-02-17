package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.TransitSubsystem;

public class SimStateMachine extends StateMachine {
    /**
     * Creates a new StateMachine
     * 
     * @param shooterSubsystem  The shooter subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param transitSubsystem  The transit subsystem
     * @param intakeSubsystem   The intake subsystem
     * @param noteDetection     The note detection subsystem
     * @param driveSubsystem    The drive subsystem
     */
    public SimStateMachine(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem,
            TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem,
            NoteDetection noteDetection, DriveSubsystem driveSubsystem) {
        super(shooterSubsystem, elevatorSubsystem, transitSubsystem, intakeSubsystem, noteDetection, driveSubsystem);
        SmartDashboard.putBoolean("Intake Limit Switch Pressed", false);
        SmartDashboard.putBoolean("Transit Limit Switch Pressed", false);
        SmartDashboard.putBoolean("Shooter At Speed", false);
        SmartDashboard.putBoolean("Elevator At Angle", false);
    }

    @Override
    protected boolean getIntakeLimitSwitch() {
        return SmartDashboard.getBoolean("Intake Limit Switch Pressed", false);
    }

    @Override
    protected boolean getTransitLimitSwitch() {
        return SmartDashboard.getBoolean("Transit Limit Switch Pressed", false);
    }

    @Override
    protected boolean shooterAtSpeed() {
        return SmartDashboard.getBoolean("Shooter At Speed", false);
    }

    @Override
    protected boolean shooterAtAngle() {
        return SmartDashboard.getBoolean("Elevator At Angle", false);
    }
}
