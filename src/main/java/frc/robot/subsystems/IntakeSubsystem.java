package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.CANConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Defines the motor
    CANSparkMax intakeTopMotor = new CANSparkMax(CANConstants.kIntakeTopMotorPort, MotorType.kBrushless);
    CANSparkMax intakeBottomMotor = new CANSparkMax(CANConstants.kIntakeBottomMotorPort, MotorType.kBrushless);
    DigitalInput intakeSensor = new DigitalInput(CANConstants.kIntakeSensorPort);

    // This will be set to true if the intake is running
    boolean isIntakeRunning = false;

    public IntakeSubsystem() {
        intakeTopMotor.restoreFactoryDefaults();
        intakeBottomMotor.restoreFactoryDefaults();
    }

    // Use this to run intake
    public void runIntake() {
        intakeTopMotor.set(1);
        intakeBottomMotor.set(-1);
    }

    // Stops intake
    private void stopIntake() {
        intakeTopMotor.set(0);
        intakeBottomMotor.set(0);
    }

    public boolean readIntakeLimitSwitch() {
        return intakeSensor.get();
    }

    public void consumeIntakeInput(boolean isTheBButtonPressed) {
        if (isTheBButtonPressed) {
            runIntake();
        } else {
            stopIntake();
        }
    }
}
