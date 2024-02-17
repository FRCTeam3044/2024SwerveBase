package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoCheckConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PDHChannelConstants;
import frc.robot.Constants.TransitConstants;

public class TransitSubsystem extends AdvancedSubsystem {
    // defines the motor and sensor
    TalonSRX transitMotor = new TalonSRX(CANConstants.kTransitMotorPort);
    DigitalInput transitSensor = new DigitalInput(7/*CANConstants.kTransitSensorPort*/);

    // if the note is in the transit then this would be true
    boolean isNoteInTransit = false;
    boolean isIntakeRunning = false;

    // change this to change the speed of the motor
    double motorSpeed = 0.8;

    public TransitSubsystem() {
        transitMotor.configFactoryDefault();
        registerHardware("Transit Motor", transitMotor);
    }

    // Use this to get the note from the intake system
    public void getNoteFromIntake() {

    }

    // Use this to send the note to the shooter
    public void sendNoteToShooter() {

    }

    // Checks the sensors every second it updates
    public boolean readTransitLimitSwitch() {
        return transitSensor.get();
    }

    public void runTransit() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed * TransitConstants.kTransitManualSpeed.get());
    }

    private void stopTransit() {
        transitMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command stopTransitCommand() {
        return Commands.runOnce(() -> stopTransit(), this);
    }

    public void consumeTransitInput(boolean isTheBButtonPressed) {
        if (isTheBButtonPressed) {
            runTransit();
        } else {
            stopTransit();
        }
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.sequence(
            Commands.runOnce(
                () -> {
                  transitMotor.set(TalonSRXControlMode.PercentOutput, 1.0);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (RobotContainer.m_powerDistroHub.getCurrent(PDHChannelConstants.TransitTalonChannel) < AutoCheckConstants.TransitTalonAmps) {
                    addFault("[System Check] Transit motor not moving", false, true);
                  }
                  transitMotor.set(TalonSRXControlMode.PercentOutput, -1.0);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (RobotContainer.m_powerDistroHub.getCurrent(PDHChannelConstants.TransitTalonChannel) < AutoCheckConstants.TransitTalonAmps) {
                    addFault("[System Check] Intake motor not moving", false, true);
                  }
                  transitMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(stopTransitCommand());
  }
}
