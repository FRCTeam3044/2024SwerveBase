package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.LimitSwitchSubsystem;

public class IntakeSubsystem extends SubsystemBase implements LimitSwitchSubsystem {
    // Defines the motor
    TalonSRX intakeTopMotor = new TalonSRX(CANConstants.kIntakeTopMotorPort);
    TalonSRX intakeBottomMotor = new TalonSRX(CANConstants.kIntakeBottomMotorPort);
    DigitalInput intakeSensor = new DigitalInput(CANConstants.kIntakeSensorPort);

    // This will be set to true if the intake is running
    boolean isIntakeRunning = false;

    public IntakeSubsystem() {
        intakeTopMotor.configFactoryDefault();
        intakeTopMotor.setInverted(true);
        intakeTopMotor.configPeakCurrentLimit(20);
        intakeBottomMotor.configFactoryDefault();
        intakeBottomMotor.configPeakCurrentLimit(20);

    }

    // Use this to run intake
    public void runIntake() {
        intakeTopMotor.set(TalonSRXControlMode.PercentOutput, 1 * IntakeConstants.kIntakeManualSpeed.get());
        intakeBottomMotor.set(TalonSRXControlMode.PercentOutput, -1 * IntakeConstants.kIntakeManualSpeed.get());
    }

    public void runIntakeReverse() {
        intakeTopMotor.set(TalonSRXControlMode.PercentOutput, -1 * IntakeConstants.kIntakeManualSpeed.get());
        intakeBottomMotor.set(TalonSRXControlMode.PercentOutput, 1 * IntakeConstants.kIntakeManualSpeed.get());
    }

    // Stops intake
    public void stopIntake() {
        intakeTopMotor.set(TalonSRXControlMode.PercentOutput, 0);
        intakeBottomMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    /**
     * Reads the intake limit switch
     * 
     * @return true if the intake limit switch is pressed
     */
    @Override
    public boolean readLimitSwitch() {
        return !intakeSensor.get();
    }

    public void consumeIntakeInput(boolean run) {
        if (run) {
            runIntake();
        } else {
            stopIntake();
        }
    }

    public void consumeIntakeInput(boolean run, boolean reverse) {
        if (run) {
            if (reverse) {
                runIntakeReverse();
            } else {
                runIntake();
            }
        } else {
            stopIntake();
        }
    }
}
