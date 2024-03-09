package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.LimitSwitchSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class IntakeSubsystem extends SubsystemBase implements LimitSwitchSubsystem {
    // Defines the motor
    TalonSRX intakeTopMotor = new TalonSRX(CANConstants.kIntakeTopMotorPort);
    TalonSRX intakeBottomMotor = new TalonSRX(CANConstants.kIntakeBottomMotorPort);
    AnalogInput intakeUltrasonic = new AnalogInput(CANConstants.kIntakeSensorPort);
    ConfigurableParameter<Integer> intakeUltrasonicThreshold = new ConfigurableParameter<Integer>(250,
            "Intake Ultrasonic Threshold");

    // This will be set to true if the intake is running
    boolean isIntakeRunning = false;

    public IntakeSubsystem() {
        intakeTopMotor.configFactoryDefault();
        intakeTopMotor.setInverted(true);
        intakeTopMotor.configPeakCurrentLimit(20);
        intakeBottomMotor.configFactoryDefault();
        intakeBottomMotor.configPeakCurrentLimit(20);
        intakeBottomMotor.setInverted(true);
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
        if (intakeUltrasonic.getValue() <= intakeUltrasonicThreshold.get()) {
            return true;
        } else {
            return false;
        }
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
