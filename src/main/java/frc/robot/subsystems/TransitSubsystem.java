package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;

public class TransitSubsystem {

    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(0);
    Ultrasonic noteDetectionSensorOne = new Ultrasonic(null, null);
    Ultrasonic noteDetectionSensorTwo = new Ultrasonic(null, null);

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

    };
}
