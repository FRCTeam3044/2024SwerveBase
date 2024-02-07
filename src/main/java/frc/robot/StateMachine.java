package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ShooterConstants;

public class StateMachine {
    public enum State {
        /**
         * Close enough to a note to pickup
         */
        NEAR_NOTE,
        /**
         * A note is in the intake but not in the transit
         */
        OWN_NOTE,
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

    private final DigitalInput intakeLimitSwitch;
    private final DigitalInput transitLimitSwitch;

    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;

    public StateMachine(DigitalInput intakeLimitSwitch, DigitalInput transitLimitSwitch,
            RelativeEncoder shooterTopEncoder, RelativeEncoder shooterBottomEncoder) {
        this.intakeLimitSwitch = intakeLimitSwitch;
        this.transitLimitSwitch = transitLimitSwitch;
        this.shooterTopEncoder = shooterTopEncoder;
        this.shooterBottomEncoder = shooterBottomEncoder;
    }

    public void periodic() {
        switch (currentState) {
            case NO_NOTE:
                // Check for near a note
                break;
            case NEAR_NOTE:
                if (intakeLimitSwitch.get()) {
                    currentState = State.OWN_NOTE;
                }
                break;
            case OWN_NOTE:
                if (transitLimitSwitch.get()) {
                    currentState = State.NOTE_LOADED;
                }
                break;
            case NOTE_LOADED:
                if (shooterAtSpeed()) {
                    currentState = State.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                break;
            default:
                currentState = State.NO_NOTE;
                break;
        }
    }

    public State getState() {
        return currentState;
    }

    private boolean shooterAtSpeed() {
        int targetVel = ShooterConstants.kShooterTargetRPM.get();
        double tolerance = ShooterConstants.kShooterToleranceRPM.get();
        return Math.abs(shooterTopEncoder.getVelocity() - targetVel) < tolerance
                && Math.abs(shooterBottomEncoder.getVelocity() - targetVel) < tolerance;
    }
}
