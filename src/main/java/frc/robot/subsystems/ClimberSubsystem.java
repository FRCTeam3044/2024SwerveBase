package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;

public class ClimberSubsystem {
    TalonSRX leftClimberMotor = new TalonSRX(0);
    TalonSRX rightClimberMotor = new TalonSRX(0);
    Encoder leftClimberEncoder = new Encoder(null, null);
    Encoder rightClimberEncoder = new Encoder(null, null);

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
}