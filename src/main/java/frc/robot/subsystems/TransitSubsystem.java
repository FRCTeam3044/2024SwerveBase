package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TransitConstants;
import frc.robot.utils.LimitSwitchSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;

public class TransitSubsystem extends SubsystemBase implements LimitSwitchSubsystem {

    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(CANConstants.kTransitMotorPort);
    AnalogInput transitUltrasonic = new AnalogInput(CANConstants.kTransitSensorPort);
    ConfigurableParameter<Integer> transitUltrasonicThreshold = new ConfigurableParameter<Integer>(250,
            "Transit Ultrasonic Threshold");

    public Timer timeSinceTransit = new Timer();
    public boolean runningTransit = false;

    public TransitSubsystem() {
        transitMotor.configFactoryDefault();
        transitMotor.setInverted(true);
        transitMotor.configPeakCurrentLimit(20);
        transitMotor.setStatusFramePeriod(2, 5000);
        transitMotor.setStatusFramePeriod(3, 5000);
        transitMotor.setStatusFramePeriod(8, 5000);
        transitMotor.setStatusFramePeriod(10, 50000);
        transitMotor.setStatusFramePeriod(12, 5000);
        transitMotor.setStatusFramePeriod(13, 5000);
        transitMotor.setStatusFramePeriod(14, 5000);
        transitMotor.setStatusFramePeriod(21, 5000);
        transitMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        transitMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 5000);
        transitMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 5000);
    }

    // Use this to get the note from the intake system
    public void getNoteFromIntake() {

    }

    // Use this to send the note to the shooter
    public void sendNoteToShooter() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Transit Ultrasonic", transitUltrasonic.getValue());
    }

    /**
     * Reads the transit limit switch
     * 
     * @return true if the transit limit switch is pressed
     */
    @Override
    public boolean readLimitSwitch() {
        if (transitUltrasonic.getValue() <= transitUltrasonicThreshold.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void runTransit() {
        if (!runningTransit) {
            runningTransit = true;
            timeSinceTransit.reset();
            timeSinceTransit.start();
        }
        transitMotor.set(TalonSRXControlMode.PercentOutput, TransitConstants.kTransitManualSpeed.get());
    }

    public void runTransitReverse() {
        if (!runningTransit) {
            runningTransit = true;
            timeSinceTransit.reset();
            timeSinceTransit.start();
        }
        transitMotor.set(TalonSRXControlMode.PercentOutput, -TransitConstants.kTransitManualSpeed.get());
    }

    public void stopTransit() {
        if (!runningTransit) {
            runningTransit = false;
            timeSinceTransit.reset();
            timeSinceTransit.start();
        }
        transitMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void consumeTransitInput(boolean run) {
        if (run) {
            runTransit();
        } else {
            stopTransit();
        }
    }

    public void consumeTransitInput(boolean run, boolean reverse) {
        if (run) {
            runTransit();
        } else if (reverse) {
            runTransitReverse();
        } else {
            stopTransit();
        }
    }
}
