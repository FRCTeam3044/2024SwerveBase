package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.commands.AutoAimCommnd;
import frc.robot.commands.DriverShootCommand;
import frc.robot.commands.IntakeCommands.IntakeRunMotorsCommand;
import frc.robot.commands.TransitCommands.TransitRunMotorCommand;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.TrackPointCommand;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.PathfindingDebugUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class StateMachine extends SubsystemBase {
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

    protected State currentState = State.NO_NOTE;

    protected final ShooterSubsystem m_shooterSubystem;
    protected final ElevatorSubsystem m_elevatorSubsystem;
    protected final TransitSubsystem m_transitSubsystem;
    protected final IntakeSubsystem m_intakeSubsystem;
    protected final NoteDetection m_noteDetection;
    protected final DriveSubsystem m_driveSubsystem;

    protected Debouncer m_intakeLimitDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    public Debouncer m_transitLimitDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    protected Debouncer m_shooterSpeedDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);
    protected Debouncer m_elevatorAngleDebouncer = new Debouncer(StateMachineConstants.kDebounce.get(),
            DebounceType.kBoth);

    // private Command currentDesiredCommand;

    public boolean changedDesiredCommand;

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
        updateDesiredCommand();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case NO_NOTE:
                if (m_noteDetection.hasNote) {
                    double distance = m_noteDetection.getClosestNoteDistance();
                    if (distance < StateMachineConstants.kNoteDetectionDistance.get()) {
                        currentState = State.TARGETING_NOTE;
                        updateDesiredCommand();
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
                if (!m_noteDetection.hasNote || m_noteDetection
                        .getClosestNoteDistance() > StateMachineConstants.kNoteDetectionDistance.get()) {
                    currentState = State.NO_NOTE;
                    updateDesiredCommand();
                }
                if (m_intakeLimitDebouncer.calculate(m_intakeSubsystem.readIntakeLimitSwitch())) {
                    currentState = State.OWNS_NOTE;
                    updateDesiredCommand();
                }
                break;
            case OWNS_NOTE:
                if (m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NOTE_LOADED;
                    updateDesiredCommand();
                }
                break;
            case NOTE_LOADED:
                boolean shooterAtSpeed = m_shooterSpeedDebouncer.calculate(m_shooterSubystem.shooterAtSpeed());
                boolean shooterAtAngle = m_elevatorAngleDebouncer.calculate(m_elevatorSubsystem.elevatorAtAngle());
                boolean inShootingZone = AutoTargetUtils.getShootingZone()
                        .isInside(new Vertex(m_driveSubsystem.getPose()));
                if (shooterAtSpeed && shooterAtAngle && inShootingZone) {
                    currentState = State.READY_TO_SHOOT;
                    updateDesiredCommand();
                }
                break;
            case READY_TO_SHOOT:
                if (!m_transitLimitDebouncer.calculate(m_transitSubsystem.readTransitLimitSwitch())) {
                    currentState = State.NO_NOTE;
                    updateDesiredCommand();
                }
                break;
            default:
                currentState = State.NO_NOTE;
                updateDesiredCommand();
                break;
        }

        SmartDashboard.putString("State", currentState.toString());

    }

    // TODO: In this case, spit out any notes we may have
    public void reset() {
        currentState = State.NO_NOTE;
        updateDesiredCommand();
    }

    public void forceState(State state) {
        currentState = state;
        updateDesiredCommand();
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
                // Feed note into shooter and shoot when driver says! (while still aiming)
                return getShootCommand();
            default:
                // Do nothing
                // TODO: Null Command handling
                return null;
        }
    }

    private Command getGoToSourceCommand() {
        Pose2d target = AutoTargetUtils.getSource();
        Pose2d trackTarget = AutoTargetUtils.getSourceTrackTarget();
        return new GoToAndTrackPointCommand(target, trackTarget, m_driveSubsystem);
    }

    private Command getPickupNoteCommand() {
        GoToNoteCommand goToNoteCommand = new GoToNoteCommand(RobotContainer.m_robotDrive,
                RobotContainer.m_noteDetection);
        IntakeRunMotorsCommand intakeRunMotorsCommand = new IntakeRunMotorsCommand(m_intakeSubsystem);
        return Commands.parallel(goToNoteCommand, intakeRunMotorsCommand);
    }

    private Command getLockinNoteCommand() {
        TransitRunMotorCommand transitRunMotorCommand = new TransitRunMotorCommand(m_transitSubsystem);
        return transitRunMotorCommand;
    }

    private Command getReadyShooterCommand() {
        ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
        if (shootingZone == null) {
            return null;
        }

        Command getToPoint;
        Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        if (shootingZone.isInside(robotPos)) {
            getToPoint = new GoToAndTrackPointCommand(m_driveSubsystem.getPose(), trackPoint, m_driveSubsystem);
        } else {
            Vertex closestPoint = shootingZone.calculateNearestPoint(robotPos);
            PathfindingDebugUtils.drawPoint("closet point", closestPoint);
            Pose2d closestPose = new Pose2d(closestPoint.x, closestPoint.y, new Rotation2d());
            getToPoint = new GoToAndTrackPointCommand(closestPose, trackPoint, m_driveSubsystem);
        }

        AutoAimCommnd autoAimCommnd = new AutoAimCommnd(m_elevatorSubsystem, m_driveSubsystem);

        return Commands.parallel(getToPoint, autoAimCommnd);
    }

    private Command getShootCommand() {
        AutoAimCommnd autoAimCommnd = new AutoAimCommnd(m_elevatorSubsystem, m_driveSubsystem);
        TrackPointCommand trackPointCommand = new TrackPointCommand(m_driveSubsystem,
                AutoTargetUtils.getShootingTarget());
        DriverShootCommand driverShootCommand = new DriverShootCommand(m_shooterSubystem, m_transitSubsystem,
                RobotContainer.m_operatorController);
        return Commands.parallel(autoAimCommnd, trackPointCommand, driverShootCommand);
    }

    protected void updateDesiredCommand() {
        // currentDesiredCommand = getCommandForState(currentState);
        changedDesiredCommand = true;
    }

    public State getState() {
        return currentState;
    }

    public Command getDesiredCommand() {
        changedDesiredCommand = false;
        return getCommandForState(currentState);
    }
}
