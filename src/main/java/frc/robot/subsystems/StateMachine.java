package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.ControllerRumble;
import frc.robot.utils.ExtraCommands;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.structures.Vertex;

public class StateMachine extends SubsystemBase {
    private ConfigurableParameter<Double> kTransitRuntime = new ConfigurableParameter<Double>(0.8, "Transit Runtime");

    public enum State {
        /**
         * Close enough to a note to pickup
         */
        TARGETING_NOTE,
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

    private static final boolean testModeEnabled = false;

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
    protected Debouncer m_hasNoteDebouncer = new Debouncer(StateMachineConstants.kDebounce.get() * 2,
            DebounceType.kBoth);
    protected Debouncer m_intakeCurrentDebouncer = new Debouncer(StateMachineConstants.kDebounce.get() / 2,
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
        if (testModeEnabled) {
            DriverStation.reportWarning("State machine is in test mode, auton function unavailable", null);
        }
        if (RobotBase.isSimulation()) {
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
                // if (getIntakeLimitSwitch()) {
                // currentState = State.NOTE_LOADED;
                // updateDesiredCommand();
                // return;
                // }

                if (m_hasNoteDebouncer.calculate(m_noteDetection.hasNote)) {
                    double distance = m_noteDetection.getClosestNoteDistance();
                    if (distance < StateMachineConstants.kNoteDetectionDistance.get()) {
                        currentState = State.TARGETING_NOTE;
                        // System.out.println("Moving to targeting note");
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
                if (!m_hasNoteDebouncer.calculate(m_noteDetection.hasNote) || m_noteDetection
                        .getClosestNoteDistance() > StateMachineConstants.kNoteDetectionDistance.get()) {
                    // System.out.println("Moving back to no note");
                    currentState = State.NO_NOTE;
                    lostNote = true;
                    updateDesiredCommand();
                }
                break;
            case NOTE_LOADED:
                if ((m_transitSubsystem.runningTransit
                        && m_transitSubsystem.timeSinceTransit.hasElapsed(getTransitRuntime()))) {
                    System.out.println("Kicking back from note loaded");
                    currentState = State.NO_NOTE;
                    updateDesiredCommand();
                    return;
                }
                boolean inShootingZone = AutoTargetUtils.getShootingZone()
                        .isInside(new Vertex(m_driveSubsystem.getPose()));
                if (!inShootingZone) {
                    Vertex closestPoint = AutoTargetUtils.getShootingZone()
                            .calculateNearestPoint(new Vertex(m_driveSubsystem.getPose()));
                    if (closestPoint.distance(m_driveSubsystem.getPose()) < 0.5) {
                        inShootingZone = true;
                    }
                }
                if (shooterAtSpeed() && /* shooterAtAngle() && */ inShootingZone) {
                    currentState = State.READY_TO_SHOOT;
                    updateDesiredCommand();
                }
                break;
            case READY_TO_SHOOT:
                boolean inShootinZone = AutoTargetUtils.getShootingZone()
                        .isInside(new Vertex(m_driveSubsystem.getPose()));
                if (!inShootinZone) {
                    Vertex closestPoint = AutoTargetUtils.getShootingZone()
                            .calculateNearestPoint(new Vertex(m_driveSubsystem.getPose()));
                    if (closestPoint.distance(m_driveSubsystem.getPose()) < 0.8) {
                        inShootinZone = true;
                    }
                }
                if ((m_transitSubsystem.runningTransit
                        && m_transitSubsystem.timeSinceTransit.hasElapsed(getTransitRuntime()))) {
                    currentState = State.NO_NOTE;
                    updateDesiredCommand();
                } else if (!inShootinZone) {
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

        Command ejectNoteCommand = Commands.deadline(Commands.waitSeconds(StateMachineConstants.kEjectTime.get()),
                m_shooterSubsystem.slow(), m_transitSubsystem.run());
        ejectNoteCommand.schedule();
        updateDesiredCommand();
    }

    public void forceState(State state) {
        currentState = state;
        updateDesiredCommand();
    }

    protected boolean getIntakeLimitSwitch() {
        return false;
        //return m_intakeLimitDebouncer.calculate(m_intakeSubsystem.readLimitSwitch());
    }

    protected boolean getTransitLimitSwitch() {
        return m_transitLimitDebouncer.calculate(m_transitSubsystem.readLimitSwitch());
    }

    protected boolean shooterAtSpeed() {
        return m_shooterSubsystem.shooterAtSpeed();
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
        // Pose2d target = AutoTargetUtils.getSource();
        // Pose2d trackTarget = AutoTargetUtils.getSourceTrackTarget();
        // return new GoToAndTrackPointCommand(target, trackTarget, m_driveSubsystem,
        // false);
        return new WaitCommand(1);
    }

    public Command getPickupNoteCommand() {
        return Commands.parallel(RobotContainer.m_robotDrive.goToNote(RobotContainer.m_noteDetection, false),
                m_intakeSubsystem.run(), m_elevatorSubsystem.intake()).withName("SM Pickup Note");
    }

    // private Command getLockinNoteCommand() {
    // Command runIntake = new
    // IntakeCommand(m_intakeSubsystem).until(this::getTransitLimitSwitch);
    // Command getToPoint = goToShootingZoneCommand();
    // if (getToPoint == null) {
    // return null;
    // }
    // return Commands.parallel(runIntake, getToPoint);
    // }

    private Command getReadyShooterCommand() {
        Command getToPoint = goToShootingZoneCommand();
        if (getToPoint == null) {
            return null;
        }
        Command runIntake = Commands.waitSeconds(1).deadlineWith(m_intakeSubsystem.run());

        return Commands.parallel(getToPoint, m_elevatorSubsystem.autoAim(m_driveSubsystem),
                m_shooterSubsystem.speaker(), runIntake);
    }

    public Command goToShootingZoneCommand(boolean forceMove) {
        // ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
        // if (shootingZone == null) {
        // return null;
        // }
        // Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        // Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        // if (!forceMove && shootingZone.isInside(robotPos)) {
        // return new TrackPointCommand(m_driveSubsystem, trackPoint, true);
        // } else {
        // Pose2d closestPoint =
        // shootingZone.calculateNearestPoint(robotPos).asPose2d();
        // GoToAndTrackPointCommand travelToPoint = new
        // GoToAndTrackPointCommand(closestPoint, trackPoint,
        // m_driveSubsystem, true);
        // TrackPointCommand trackPointCmd = new TrackPointCommand(m_driveSubsystem,
        // trackPoint, true);
        // return travelToPoint.andThen(trackPointCmd);
        // }
        return new WaitCommand(0.1);
    }

    public Command goToShootingZoneCommand() {
        return goToShootingZoneCommand(false);
    }

    private Command getShootCommand() {

        AtomicBoolean hasShot = new AtomicBoolean(false);
        // Yes this is disgusting, I will brainstorm better ways to do the state machine
        // but this is a quick workaround until we have a more concrete plan.
        Command driverShootCommand = Commands.race(m_shooterSubsystem.speaker(), ExtraCommands.runUntil(() -> {
            if (RobotContainer.m_operatorController.getRightTriggerAxis() > 0.5 && !hasShot.get()) {
                hasShot.set(true);
                ControllerRumble.driverSmallShort();
                ControllerRumble.opSmallShort();
                m_shooterSubsystem.saveShotData();
            }
            if (hasShot.get()) {
                if (m_shooterSubsystem.shooterAtSpeed()) {
                    m_transitSubsystem.runTransit();
                }
            }
        }, () -> !m_transitLimitDebouncer.calculate(m_transitSubsystem.readLimitSwitch()) && hasShot.get(),
                m_transitSubsystem)).withName("SM Shoot");
        return Commands.parallel(m_elevatorSubsystem.autoAim(m_driveSubsystem),
                m_driveSubsystem.trackPoint(AutoTargetUtils.getShootingTarget(), true), driverShootCommand);
    }

    @SuppressWarnings("unused")
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

    private double getTransitRuntime() {
        // if (DriverStation.isTeleop()) {
        // return kTeleTransitRuntime.get();
        // } else {
        // return kTransitRuntime.get();
        // }
        double runtime = kTransitRuntime.get();
        // System.out.println("transit runtime " + runtime);
        return runtime;
    }
}
