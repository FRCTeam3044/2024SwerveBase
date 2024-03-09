package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriverShootCommand;
import frc.robot.commands.ShooterSlowCommand;
import frc.robot.commands.SpeakerShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeRunUntilSwitch;
import frc.robot.commands.TransitCommands.TransitCommand;
import frc.robot.commands.TransitCommands.TransitRunMotorCommand;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.TrackPointCommand;
import frc.robot.utils.AutoTargetUtils;
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

    private static final boolean testModeEnabled = true;

    protected State currentState = State.NO_NOTE;

    protected final ShooterSubsystem m_shooterSubsystem;
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

    public boolean lostNote = false;

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
        m_shooterSubsystem = shooterSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_transitSubsystem = transitSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_noteDetection = noteDetection;
        m_driveSubsystem = driveSubsystem;
        updateDesiredCommand();

        if (testModeEnabled) {
            SmartDashboard.putString("State Machine/Override", "NO_NOTE");
            SmartDashboard.putBoolean("State Machine/Allow State Navigation", false);
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            SmartDashboard.putBoolean("Intake Limit Switch", RobotContainer.m_driverController.getHID().getAButton());
            SmartDashboard.putBoolean("Transit Limit Switch", RobotContainer.m_driverController.getHID().getBButton());
            SmartDashboard.putBoolean("Shooter At Speed", RobotContainer.m_driverController.getHID().getXButton());
            SmartDashboard.putBoolean("Shooter At Angle", RobotContainer.m_driverController.getHID().getYButton());
        }

        if (testModeEnabled) {
            String state = SmartDashboard.getString("State Machine/Override", "NO_NOTE");
            State newState;
            try {
                newState = State.valueOf(state);
            } catch (IllegalArgumentException e) {
                return;
            }
            if (newState != currentState) {
                forceState(newState);
                updateDesiredCommand();
            }
            if (!SmartDashboard.getBoolean("State Machine/Allow State Navigation", false)) {
                return;
            }
        }

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
                    lostNote = true;
                    updateDesiredCommand();
                }
                if (getIntakeLimitSwitch()) {
                    currentState = State.OWNS_NOTE;
                    updateDesiredCommand();
                }
                break;
            case OWNS_NOTE:
                if (getTransitLimitSwitch()) {
                    currentState = State.NOTE_LOADED;
                    updateDesiredCommand();
                }
                break;
            case NOTE_LOADED:
                boolean inShootingZone = AutoTargetUtils.getShootingZone()
                        .isInside(new Vertex(m_driveSubsystem.getPose()));
                if (shooterAtSpeed() && shooterAtAngle() && inShootingZone) {
                    currentState = State.READY_TO_SHOOT;
                    updateDesiredCommand();
                }
                break;
            case READY_TO_SHOOT:
                if (!getTransitLimitSwitch()) {
                    currentState = State.NO_NOTE;
                    updateDesiredCommand();
                } else if (!AutoTargetUtils.getShootingZone().isInside(new Vertex(m_driveSubsystem.getPose()))) {
                    currentState = State.NOTE_LOADED;
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

    public void reset() {
        currentState = State.NO_NOTE;

        Command ejectNoteCommand = Commands.deadline(new WaitCommand(StateMachineConstants.kEjectTime.get()),
                new ShooterSlowCommand(m_shooterSubsystem),
                new TransitCommand(m_transitSubsystem));
        ejectNoteCommand.schedule();
        updateDesiredCommand();
    }

    public void forceState(State state) {
        currentState = state;
        updateDesiredCommand();
    }

    protected boolean getIntakeLimitSwitch() {
        return m_intakeLimitDebouncer.calculate(m_intakeSubsystem.readLimitSwitch());
    }

    protected boolean getTransitLimitSwitch() {
        return m_transitLimitDebouncer.calculate(m_transitSubsystem.readLimitSwitch());
    }

    protected boolean shooterAtSpeed() {
        return m_shooterSpeedDebouncer.calculate(m_shooterSubsystem.shooterAtSpeed());
    }

    protected boolean shooterAtAngle() {
        return m_elevatorAngleDebouncer.calculate(m_elevatorSubsystem.elevatorAtAngle());
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
                RobotContainer.m_noteDetection, false);
        IntakeRunUntilSwitch intakeRunMotorsCommand = new IntakeRunUntilSwitch(m_intakeSubsystem);
        return Commands.parallel(goToNoteCommand, intakeRunMotorsCommand);
    }

    private Command getLockinNoteCommand() {
        TransitRunMotorCommand transitRunMotorCommand = new TransitRunMotorCommand(m_transitSubsystem);
        Command getToPoint = goToShootingZoneCommand();
        if (getToPoint == null) {
            return null;
        }
        return Commands.parallel(transitRunMotorCommand, getToPoint);
    }

    private Command getReadyShooterCommand() {
        Command getToPoint = goToShootingZoneCommand();
        if (getToPoint == null) {
            return null;
        }
        AutoAimCommand autoAimCommand = new AutoAimCommand(m_elevatorSubsystem, m_driveSubsystem);
        SpeakerShooterCommand speakerShooterCommand = new SpeakerShooterCommand(m_shooterSubsystem);

        return Commands.parallel(getToPoint, autoAimCommand, speakerShooterCommand);
    }

    public Command goToShootingZoneCommand() {
        ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
        if (shootingZone == null) {
            return null;
        }
        Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        if (shootingZone.isInside(robotPos)) {
            return new TrackPointCommand(m_driveSubsystem, trackPoint);
        } else {
            Pose2d closestPoint = shootingZone.calculateNearestPoint(robotPos).asPose2d();
            GoToAndTrackPointCommand travelToPoint = new GoToAndTrackPointCommand(closestPoint, trackPoint,
                    m_driveSubsystem);
            TrackPointCommand trackPointCmd = new TrackPointCommand(m_driveSubsystem, trackPoint);
            return travelToPoint.andThen(trackPointCmd);
        }
    }

    private Command getShootCommand() {
        AutoAimCommand autoAimCommand = new AutoAimCommand(m_elevatorSubsystem, m_driveSubsystem);
        TrackPointCommand trackPointCommand = new TrackPointCommand(m_driveSubsystem,
                AutoTargetUtils.getShootingTarget());
        DriverShootCommand driverShootCommand = new DriverShootCommand(m_shooterSubsystem, m_transitSubsystem,
                RobotContainer.m_operatorController);
        return Commands.parallel(autoAimCommand, trackPointCommand, driverShootCommand);
    }

    protected void updateDesiredCommand() {
        // currentDesiredCommand = getCommandForState(currentState);
        if (testModeEnabled && SmartDashboard.getBoolean("State Machine/Allow State Navigation", false)) {
            SmartDashboard.putString("State Machine/Override", currentState.toString());
        }
        changedDesiredCommand = true;
    }

    public State getState() {
        return currentState;
    }

    public Command getDesiredCommand() {
        Command cmd = getCommandForState(currentState);
        if (cmd != null) {
            changedDesiredCommand = false;
        }
        return cmd;
    }
}
