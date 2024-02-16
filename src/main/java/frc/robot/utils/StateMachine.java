package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.commands.AutoAimCommnd;
import frc.robot.commands.IntakeCommands.IntakeRunMotorsCommand;
import frc.robot.commands.TransitCommands.TransitRunMotorCommand;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.GoToPointDriverRotCommand;
import frc.robot.commands.drive.TrackPointCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import me.nabdev.pathfinding.structures.Obstacle;
import me.nabdev.pathfinding.structures.Vertex;

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
    private IntakeSubsystem m_intakeSubsystem;
    private NoteDetection m_noteDetection;
    private DriveSubsystem m_driveSubsystem;

    private Debouncer m_intakeLimitDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_transitLimitDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_shooterSpeedDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    private Debouncer m_elevatorAngleDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);

    private Command currentDesiredCommand = getCommandForState(currentState);

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
    public StateMachine(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem,
            TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem,
            NoteDetection noteDetection, DriveSubsystem driveSubsystem) {
        m_shooterSubystem = shooterSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_transitSubsystem = transitSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_noteDetection = noteDetection;
        m_driveSubsystem = driveSubsystem;
    }

    public void periodic() {
        switch (currentState) {
            case NO_NOTE:
                if (m_noteDetection.hasNote) {
                    double distance = m_noteDetection.getClosestNoteDistance();
                    if (distance < StateMachineConstants.kNoteDetectionDistance.get()) {
                        currentState = State.TARGETING_NOTE;
                        currentDesiredCommand = getCommandForState(currentState);
                    }
                }
                break;
            case TARGETING_NOTE:
                /*
                 * TODO: When alex finishes the note detection, we will need a clean way to
                 * handle resetting the state General plan for that: - If the note is no longer
                 * detected, reset the state to NO_NOTE, and let it decide if its close enough
                 * to a note to pickup. Rumble the controller to let the driver know that the
                 * robot is no longer targeting a note.
                 */
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
                return getReadyShooterCommand();
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
        Pose2d target = AutoTargetUtils.getSource();
        return new GoToPointDriverRotCommand(target, m_driveSubsystem, RobotContainer.m_driverController);
    }

    private Command getPickupNoteCommand() {
        GoToNoteCommand goToNoteCommand = new GoToNoteCommand(RobotContainer.m_robotDrive,
                RobotContainer.m_noteDetection);
        IntakeRunMotorsCommand intakeRunMotorsCommand = new IntakeRunMotorsCommand(m_intakeSubsystem);
        return new ParallelCommandGroup(goToNoteCommand, intakeRunMotorsCommand);
    }

    private Command getLockinNoteCommand() {
        TransitRunMotorCommand transitRunMotorCommand = new TransitRunMotorCommand(m_transitSubsystem);
        return transitRunMotorCommand;
    }

    private Command getReadyShooterCommand() {
        Obstacle shootingZone = AutoTargetUtils.getShootingZone();
        if (shootingZone == null) {
            return null;
        }

        Command getToPoint;
        Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        if (shootingZone.isInside(robotPos)) {
            getToPoint = new TrackPointCommand(m_driveSubsystem, trackPoint);
        } else {
            Vertex closestPoint = shootingZone.calculateNearestPoint(robotPos);
            Pose2d closestPose = new Pose2d(closestPoint.x, closestPoint.y, new Rotation2d());
            getToPoint = new GoToAndTrackPointCommand(closestPose, trackPoint, m_driveSubsystem);
        }

        AutoAimCommnd autoAimCommnd = new AutoAimCommnd(m_elevatorSubsystem, m_driveSubsystem);

        return new ParallelCommandGroup(getToPoint, autoAimCommnd);
    }

    public State getState() {
        return currentState;
    }

    public Command getDesiredCommand() {
        return currentDesiredCommand;
    }
}
