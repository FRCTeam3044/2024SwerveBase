package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathfindingTargets;
import frc.robot.commands.IntakeCommands.IntakeRunMotorsCommand;
import frc.robot.commands.TransitCommands.TransitRunMotorCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.GoToPointDriverRotCommand;
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

    private Debouncer m_noteDetectionDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_intakeLimitDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_transitLimitDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_shooterSpeedDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_elevatorAngleDebouncer = new Debouncer(DriveConstants.kStatemachineDebounce.get(),
            DebounceType.kBoth);

    private Command currentDesiredCommand = getCommandForState(currentState);

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
                    currentDesiredCommand = getCommandForState(currentState);
                }
                break;
            case TARGETING_NOTE:
                if (m_intakeLimitDebouncer.calculate(m_intakeSubsystem.readIntakeLimitSwitch())) {
                    currentState = State.OWNS_NOTE;
                    currentDesiredCommand = getCommandForState(currentState);
                }
                break;
            case OWNS_NOTE:
                if (m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NOTE_LOADED;
                    currentDesiredCommand = getCommandForState(currentState);
                }
                break;
            case NOTE_LOADED:
                boolean shooterAtSpeed = m_shooterSpeedDebouncer.calculate(m_shooterSubystem.shooterAtSpeed());
                boolean shooterAtAngle = m_elevatorAngleDebouncer.calculate(m_elevatorSubsystem.elevatorAtAngle());
                if (shooterAtSpeed && shooterAtAngle) {
                    currentState = State.READY_TO_SHOOT;
                    currentDesiredCommand = getCommandForState(currentState);
                }
                break;
            case READY_TO_SHOOT:
                if (!m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NO_NOTE;
                    currentDesiredCommand = getCommandForState(currentState);
                }
            default:
                currentState = State.NO_NOTE;
                currentDesiredCommand = getCommandForState(currentState);
                break;
        }

        SmartDashboard.putString("State", currentState.toString());
    }

    private Command getCommandForState(State state) {
        switch (state) {
            case NO_NOTE:
                // Go To Source
                return getGoToSourceCommand();
            case TARGETING_NOTE:
                // Go To & Pickup Note
                return getPickupNoteCommand();
            case OWNS_NOTE:
                // Get note locked in transit
                return getLockinNoteCommand();
            case NOTE_LOADED:
                // Start Aiming shooter and speeding up wheels
                // TODO: Auto Aiming
                return null;
            case READY_TO_SHOOT:
                // Feed note into shooter and shoot! (while still aiming)
                // TODO: Auto Aiming
                return null;
            default:
                // Do nothing
                // TODO: Null Command handling
                return null;
        }
    }

    private Command getGoToSourceCommand() {
        Pose2d target;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return null;
        }
        if (alliance.get() == Alliance.Red) {
            target = PathfindingTargets.RED_SOURCE;
        } else {
            target = PathfindingTargets.BLUE_SOURCE;
        }
        return new GoToPointDriverRotCommand(target, m_driveSubsystem, RobotContainer.m_driverController);
    }

    private Command getPickupNoteCommand() {
        GoToNoteCommand goToNoteCommand = new GoToNoteCommand(RobotContainer.m_robotDrive,
                RobotContainer.m_noteDetection);
        IntakeRunMotorsCommand intakeRunMotorsCommand = new IntakeRunMotorsCommand(m_intakeSubsystem);
        return new ParallelCommandGroup(goToNoteCommand, intakeRunMotorsCommand);
    }

    // TODO:
    private Command getLockinNoteCommand() {
        TransitRunMotorCommand transitRunMotorCommand = new TransitRunMotorCommand(m_transitSubsystem);
        return transitRunMotorCommand;
    }

    public State getState() {
        return currentState;
    }

    public Command getDesiredCommand() {
        return currentDesiredCommand;
    }
}
