package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitSubsystem extends SubsystemBase {

    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(0);
    DigitalInput intakeSensorOne = new DigitalInput(0);
    DigitalInput intakeSnsorTwo = new DigitalInput(0);

    // if the note is in the transit then this would be true
    boolean isNoteInTransit = false;
    boolean isIntakeRunning = false;

    // change this to change the speed of the motor
    double motorSpeed = 0;

    // Use this to get the note from the intake system
    public void getNoteFromIntake() {

    }

    // Use this to send the note to the shooter
    public void sendNoteToShooter() {

    }

    // Checks the sensors every second it updates
    public void updatePeriodic() {
        boolean hasNoteHitFirstSensor = intakeSensorOne.get();
        boolean hasNoteHitSecondSensor = intakeSnsorTwo.get();
    }

    private void runTransit() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }

    private void stopTransit() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void consumeTransitInput(boolean isTheBButtonPressed) {
        if (isTheBButtonPressed) {
            runTransit();
        } else {
            stopTransit();
        }
    }
}
