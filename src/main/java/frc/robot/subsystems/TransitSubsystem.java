package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;

public class TransitSubsystem {

    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(0);
    Ultrasonic noteDetection = new Ultrasonic(null, null);

    // if the note is in the transit then this would be true
    boolean isNoteInTransit = false;

    double motorSpeed = 0;

    // Use this to make the transit move
    public void startTransit() {

    };
}
