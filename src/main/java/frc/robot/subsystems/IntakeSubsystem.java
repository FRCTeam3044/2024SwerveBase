package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;
import me.nabdev.oxconfig.ConfigurableParameter;

public class IntakeSubsystem extends SubsystemBase {
    // Defines the motor
    TalonSRX intakeTopMotor = new TalonSRX(CANConstants.kIntakeTopMotorPort);
    TalonSRX intakeBottomMotor = new TalonSRX(CANConstants.kIntakeBottomMotorPort);
    AnalogInput intakeUltrasonic = new AnalogInput(CANConstants.kIntakeSensorPort);
    ConfigurableParameter<Integer> intakeUltrasonicThreshold = new ConfigurableParameter<Integer>(250,
            "Intake Ultrasonic Threshold");
    ConfigurableParameter<Double> intakeSpinupSpeed = new ConfigurableParameter<Double>(0.4,
            "Intake Spinup Speed");
    ConfigurableParameter<Double> kIntakeCurrentThreshold = new ConfigurableParameter<Double>(-16.0,
            "Intake Current Threshold");

    // This will be set to true if the intake is running
    public boolean isIntakeRunning = false;

    public Timer timeSinceStart = new Timer();

    private boolean hasNote = false;

    public IntakeSubsystem() {
        intakeTopMotor.configFactoryDefault();
        intakeTopMotor.setInverted(true);
        intakeTopMotor.configPeakCurrentLimit(20);
        intakeTopMotor.enableCurrentLimit(true);
        intakeBottomMotor.configFactoryDefault();
        intakeBottomMotor.configPeakCurrentLimit(20);
        intakeBottomMotor.enableCurrentLimit(true);
        intakeBottomMotor.setInverted(true);

        intakeBottomMotor.setStatusFramePeriod(2, 5000);
        intakeBottomMotor.setStatusFramePeriod(3, 5000);
        intakeBottomMotor.setStatusFramePeriod(8, 5000);
        intakeBottomMotor.setStatusFramePeriod(10, 50000);
        intakeBottomMotor.setStatusFramePeriod(12, 5000);
        intakeBottomMotor.setStatusFramePeriod(13, 5000);
        intakeBottomMotor.setStatusFramePeriod(14, 5000);
        intakeBottomMotor.setStatusFramePeriod(21, 5000);
        intakeBottomMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        intakeBottomMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 5000);
        intakeBottomMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 5000);

        intakeTopMotor.setStatusFramePeriod(2, 5000);
        intakeTopMotor.setStatusFramePeriod(3, 5000);
        intakeTopMotor.setStatusFramePeriod(8, 5000);
        intakeTopMotor.setStatusFramePeriod(10, 50000);
        intakeTopMotor.setStatusFramePeriod(12, 5000);
        intakeTopMotor.setStatusFramePeriod(13, 5000);
        intakeTopMotor.setStatusFramePeriod(14, 5000);
        intakeTopMotor.setStatusFramePeriod(21, 5000);
        intakeTopMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        intakeTopMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 5000);
        intakeTopMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 5000);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake Ultrasonic", intakeUltrasonic.getValue());
        // SmartDashboard.putNumber("Intake Current",
        // intakeBottomMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake Top Current", -intakeTopMotor.getStatorCurrent());

        if (getCurrent() < kIntakeCurrentThreshold.get()) {
            hasNote = true;
        }
    }


    public double getCurrent() {
        return -intakeTopMotor.getStatorCurrent();
    }

    // Use this to run intake
    private void runIntake(double output) {
        intakeTopMotor.set(TalonSRXControlMode.PercentOutput, output);
        intakeBottomMotor.set(TalonSRXControlMode.PercentOutput, -output);
    }

    // Command Factories

    public Command run() {
        return run(false);
    }

    public Command run(boolean reverse) {
        return new Command() {
            {
                addRequirements(IntakeSubsystem.this);
                setName("Intake Command");
            }

            @Override
            public void execute() {
                double output = Math.min(timeSinceStart.get() * intakeSpinupSpeed.get(),
                        IntakeConstants.kIntakeManualSpeed.get());
                runIntake(reverse ? -output : output);
            }

            @Override
            public void end(boolean interrupted) {
                runIntake(0);
            }
        };
    }

    // Functional Command Variation
    // public Command run(boolean reverse) {
    // return new FunctionalCommand(
    // /* Initialize */ () -> {
    // timeSinceStart.restart();
    // isIntakeRunning = true;
    // }, /* Execute */ () -> {
    // double output = Math.min(timeSinceStart.get() * intakeSpinupSpeed.get(),
    // IntakeConstants.kIntakeManualSpeed.get());
    // runIntake(reverse ? -output : output);
    // },
    // /* End */ (interrupted) -> runIntake(0), /* IsFinished */ () -> false, this)
    // .withName("runIntake");
    // }

    public void setNoteDropped() {
        hasNote = false;
    }

    public boolean hasNote() {
        return hasNote;
    }
}
