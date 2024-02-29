package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonSRX leftClimberMotor = new TalonSRX(CANConstants.kClimberLeftClimberMotorPort);
    TalonSRX rightClimberMotor = new TalonSRX(CANConstants.kClimberRightClimberMotorPort);

    public ClimberSubsystem() {
        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();
    }

    // contorls left arm
    public void leftArm(double moveLeftMotorPower) {
        rightClimberMotor.set(TalonSRXControlMode.PercentOutput, moveLeftMotorPower);
    }

    // controls right arm
    public void rightArm(double moveRightMotorPower) {
        rightClimberMotor.set(TalonSRXControlMode.PercentOutput, moveRightMotorPower);
    }

    public void consumeClimberInput(double leftPow, double rightPow) {
        leftArm(leftPow);
        rightArm(rightPow);
    }

}