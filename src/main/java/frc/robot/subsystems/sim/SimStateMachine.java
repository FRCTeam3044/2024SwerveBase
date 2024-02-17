package frc.robot.subsystems.sim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.Vertex;

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
    public void periodic() {
        switch (this.currentState) {
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
                if (SmartDashboard.getBoolean("Intake Limit Switch Pressed", false)) {
                    currentState = State.OWNS_NOTE;
                    updateDesiredCommand();
                }
                break;
            case OWNS_NOTE:
                if (SmartDashboard.getBoolean("Transit Limit Switch Pressed", false)) {
                    currentState = State.NOTE_LOADED;
                    updateDesiredCommand();
                }
                break;
            case NOTE_LOADED:
                boolean shooterAtSpeed = SmartDashboard.getBoolean("Shooter At Speed", false);
                boolean shooterAtAngle = SmartDashboard.getBoolean("Elevator At Angle", false);
                boolean inShootingZone = AutoTargetUtils.getShootingZone()
                        .isInside(new Vertex(m_driveSubsystem.getPose()));
                if (shooterAtSpeed && shooterAtAngle && inShootingZone) {
                    currentState = State.READY_TO_SHOOT;
                    updateDesiredCommand();
                }
                break;
            case READY_TO_SHOOT:
                if (!SmartDashboard.getBoolean("Transit Limit Switch Pressed", false)) {
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
}
