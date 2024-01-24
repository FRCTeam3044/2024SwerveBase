package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem {
    // Defines the motor
    CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);

    // This will be set to true if the intake is running
    boolean isIntakeRunning = false;

    // Use this to run intake
    public void runIntake() {

    }

    // Stops intake
    public void stopIntake() {

    }
}
