package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

        leftClimberMotor.configPeakCurrentLimit(30);
        rightClimberMotor.configPeakCurrentLimit(30);

        leftClimberMotor.setStatusFramePeriod(2, 5000);
        leftClimberMotor.setStatusFramePeriod(3, 5000);
        leftClimberMotor.setStatusFramePeriod(8, 5000);
        leftClimberMotor.setStatusFramePeriod(10, 50000);
        leftClimberMotor.setStatusFramePeriod(12, 5000);
        leftClimberMotor.setStatusFramePeriod(13, 5000);
        leftClimberMotor.setStatusFramePeriod(14, 5000);
        leftClimberMotor.setStatusFramePeriod(21, 5000);
        leftClimberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        leftClimberMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 5000);
        leftClimberMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 5000);

        rightClimberMotor.setStatusFramePeriod(2, 5000);
        rightClimberMotor.setStatusFramePeriod(3, 5000);
        rightClimberMotor.setStatusFramePeriod(8, 5000);
        rightClimberMotor.setStatusFramePeriod(10, 50000);
        rightClimberMotor.setStatusFramePeriod(12, 5000);
        rightClimberMotor.setStatusFramePeriod(13, 5000);
        rightClimberMotor.setStatusFramePeriod(14, 5000);
        rightClimberMotor.setStatusFramePeriod(21, 5000);
        rightClimberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        rightClimberMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 5000);
        rightClimberMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 5000);

    }

    // contorls left arm
    public void leftArm(double moveLeftMotorPower) {
        leftClimberMotor.set(TalonSRXControlMode.PercentOutput, moveLeftMotorPower);
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