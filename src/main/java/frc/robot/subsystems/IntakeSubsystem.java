package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem {
    // Defines the motor
    CANSparkMax intakeTopMotor = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax intakeBottomMotor = new CANSparkMax(0, MotorType.kBrushless);

    // This will be set to true if the intake is running
    boolean isIntakeRunning = false;

    // Use this to run intake
    private void runIntake() {
        intakeTopMotor.set(1);
        intakeBottomMotor.set(-1);
    }

    // Stops intake
    private void stopIntake() {
        intakeTopMotor.set(0);
        intakeBottomMotor.set(0);
    }

    public void consumeIntakeInput(boolean isTheBButtonPressed) {
        if (isTheBButtonPressed) {
            runIntake();
        }
        else {
            stopIntake();
        }
    }
}
