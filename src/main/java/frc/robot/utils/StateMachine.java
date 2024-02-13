package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;

public class StateMachine {
    public enum State {
        /**
         * Close enough to a note to pickup
         */
        TARGETING_NOTE,
        /**
         * A note is in the intake but not in the transit
         */
        OWNS_NOTE,
        /**
         * A note is in the transit & ready to be shot
         */
        NOTE_LOADED,
        /**
         * Note is loaded and wheels are up to speed
         */
        READY_TO_SHOOT,
        /**
         * Robot does have a note and is not near a note
         */
        NO_NOTE
    }

    private State currentState = State.NO_NOTE;

    private ShooterSubsystem m_shooterSubystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private TransitSubsystem m_transitSubsystem;
    private ClimberSubsystem m_climberSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private NoteDetection m_noteDetection;
    private DriveSubsystem m_driveSubsystem;

    Debouncer m_noteDetectionDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(), DebounceType.kBoth);
    Debouncer m_intakeLimitDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(), DebounceType.kBoth);
    Debouncer m_transitLimitDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(), DebounceType.kBoth);
    Debouncer m_shooterSpeedDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(), DebounceType.kBoth);

    public StateMachine(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem,
            TransitSubsystem transitSubsystem, ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem,
            NoteDetection noteDetection, DriveSubsystem driveSubsystem) {
        m_shooterSubystem = shooterSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_transitSubsystem = transitSubsystem;
        m_climberSubsystem = climberSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_noteDetection = noteDetection;
        m_driveSubsystem = driveSubsystem;
    }

    public void periodic() {
        switch (currentState) {
            case NO_NOTE:
                // TODO: This will likely need to change when alex refactors the note detector
                if (m_noteDetectionDebouncer.calculate(m_noteDetection.hasNote)) {
                    currentState = State.TARGETING_NOTE;
                }
                break;
            case TARGETING_NOTE:
                if (m_intakeLimitDebouncer.calculate(m_intakeSubsystem.readIntakeLimitSwitch())) {
                    currentState = State.OWNS_NOTE;
                }
                break;
            case OWNS_NOTE:
                if (m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NOTE_LOADED;
                }
                break;
            case NOTE_LOADED:
                if (m_shooterSpeedDebouncer.calculate(m_shooterSubystem.shooterAtSpeed())) {
                    currentState = State.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                if (!m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NO_NOTE;
                }
            default:
                currentState = State.NO_NOTE;
                break;
        }

        SmartDashboard.putString("State", currentState.toString());
    }

    public State getState() {
        return currentState;
    }

}
