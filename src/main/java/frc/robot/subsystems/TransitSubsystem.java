package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
    private ConfigurableParameter<Double> kTransitRuntime = new ConfigurableParameter<Double>(0.8, "Transit Runtime");

    public Timer timeSinceTransit = new Timer();
    public boolean runningTransit = false;

    public TransitSubsystem() {
        transitMotor.configFactoryDefault();
        transitMotor.setInverted(true);
        transitMotor.configPeakCurrentLimit(20);
        transitMotor.enableCurrentLimit(true);
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

    @Override
    public void periodic() {
        if (runningTransit && timeSinceTransit.hasElapsed(kTransitRuntime.get())) {
            RobotContainer.intake.setNoteDropped();
        }
        if (RobotBase.isSimulation() && RobotContainer.m_driverController.getHID().getYButtonPressed()) {
            RobotContainer.intake.setNoteDropped();
        }
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
        runningTransit = false;
        transitMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command run() {
        return Commands.runEnd(() -> {
            runTransit();
        }, () -> {
            stopTransit();
        }, this).withName("Run Transit");
    }

    public Command runUntilSwitch() {
        return run().until(this::readLimitSwitch);
    }
}
