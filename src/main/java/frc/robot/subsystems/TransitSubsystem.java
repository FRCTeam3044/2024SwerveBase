package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TransitConstants;
import frc.robot.utils.LimitSwitchSubsystem;

public class TransitSubsystem extends SubsystemBase implements LimitSwitchSubsystem {

    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(CANConstants.kTransitMotorPort);
    public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
    public AnalogInput ultrasonicSensorTwo = new AnalogInput(1);
    public int voltageScaleFactor = 1;

    public double ultrasonicSensorOneOutput = 0;
    public double ultrasonicSensorTwoOutput = 0;

    // if the note is in the transit then this would be true
    boolean isNoteInTransit = false;
    boolean isIntakeRunning = false;

    public TransitSubsystem() {
        transitMotor.configFactoryDefault();
        transitMotor.setInverted(true);
        transitMotor.configPeakCurrentLimit(20);
        transitMotor.setStatusFramePeriod(2, 2000);
        transitMotor.setStatusFramePeriod(3, 2000);
        transitMotor.setStatusFramePeriod(8, 2000);
        transitMotor.setStatusFramePeriod(10, 20000);
        transitMotor.setStatusFramePeriod(12, 2000);
        transitMotor.setStatusFramePeriod(13, 2000);
        transitMotor.setStatusFramePeriod(14, 2000);
        transitMotor.setStatusFramePeriod(21, 2000);
        transitMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 2000);
        transitMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 2000);
        transitMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 2000);
    }

    // Use this to get the note from the intake system
    public void getNoteFromIntake() {

    }

    // Use this to send the note to the shooter
    public void sendNoteToShooter() {

    }

    /**
     * Reads the transit limit switch
     * 
     * @return true if the transit limit switch is pressed
     */
    @Override
    public boolean readLimitSwitch() {
        return !transitSensor.get();
    }

    public void runTransit() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, TransitConstants.kTransitManualSpeed.get());
    }

    public void runTransitReverse() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, -TransitConstants.kTransitManualSpeed.get());
    }

    public void stopTransit() {
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
