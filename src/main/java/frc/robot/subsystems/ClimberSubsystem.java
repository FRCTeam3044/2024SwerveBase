package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonSRX leftClimberMotor = new TalonSRX(CANConstants.kClimberLeftClimberMotor);
    TalonSRX rightClimberMotor = new TalonSRX(CANConstants.kClimberRightClimberMotor);
    // Encoder leftClimberEncoder = new Encoder(null, null);
    // Encoder rightClimberEncoder = new Encoder(null, null);
    
    // power going into each of the motors
    double leftMotorPower = 0;
    double rightMotorPower = 0;

    // contorls left arm
    public void leftArm(double moveLeftMotorPower) {
        moveLeftMotorPower = leftMotorPower;
        rightClimberMotor.set(TalonSRXControlMode.PercentOutput, moveLeftMotorPower);
    }

    // controls right arm
    public void rightArm(double moveRightMotorPower) {
        moveRightMotorPower = rightMotorPower;
        rightClimberMotor.set(TalonSRXControlMode.PercentOutput, moveRightMotorPower);
    }

    public void consumeClimberInput(boolean isLeftBumperPressed, boolean isRightBumperPressed, double rightYValue) {
        if (isLeftBumperPressed) {
            leftArm(rightYValue);
        } else {
            leftArm(0);
        }

        if (isRightBumperPressed) {
            rightArm(rightYValue);
        } else {
            rightArm(0);
        }
    }
}