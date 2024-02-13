package frc.robot.utils;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public StateMachine(DigitalInput intakeLimitSwitch, DigitalInput transitLimitSwitch) {
        this.intakeLimitSwitch = intakeLimitSwitch;
        this.transitLimitSwitch = transitLimitSwitch;
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
                // if (shooterAtSpeed()) {
                // currentState = State.READY_TO_SHOOT;
                // }
                break;
            case READY_TO_SHOOT:
                break;
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
