package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonSRX leftClimberMotor = new TalonSRX(CANConstants.kClimberLeftClimberMotorPort);
    TalonSRX rightClimberMotor = new TalonSRX(CANConstants.kClimberRightClimberMotorPort);

    public ClimberSubsystem() {
        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();

        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);

        leftClimberMotor.configPeakCurrentLimit(40);
        leftClimberMotor.enableCurrentLimit(true);
        rightClimberMotor.configPeakCurrentLimit(40);
        leftClimberMotor.enableCurrentLimit(true);
        rightClimberMotor.setInverted(true);

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
    private void leftArm(double moveLeftMotorPower) {
        leftClimberMotor.set(TalonSRXControlMode.PercentOutput, moveLeftMotorPower);
    }

    // controls right arm
    private void rightArm(double moveRightMotorPower) {
        rightClimberMotor.set(TalonSRXControlMode.PercentOutput, moveRightMotorPower);
    }

    public Command moveClimber(DoubleSupplier leftMotorPower, DoubleSupplier rightMotorPower) {
        return Commands.runEnd(() -> {
            leftArm(leftMotorPower.getAsDouble());
            rightArm(rightMotorPower.getAsDouble());
        }, () -> {
            leftArm(0);
            rightArm(0);
        }, this).withName("Move Climber");
    }

    public Command moveLeftClimber(DoubleSupplier power) {
        return Commands.runEnd(() -> {
            leftArm(power.getAsDouble());
        }, () -> {
            leftArm(0);
        }, this).withName("Move Left Climber");
    }

    public Command moveRightClimber(DoubleSupplier power) {
        return Commands.runEnd(() -> {
            rightArm(power.getAsDouble());
        }, () -> {
            rightArm(0);
        }, this).withName("Move Right Climber");
    }

    public Command moveClimberJoysticks(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
        DoubleSupplier rightStick = () -> -MathUtil.applyDeadband(rightJoystick.getAsDouble(),
                ClimberConstants.kClimberControlDeadband.get()) * ClimberConstants.kClimberManualSpeed.get();
        DoubleSupplier leftStick = () -> -MathUtil.applyDeadband(leftJoystick.getAsDouble(),
                ClimberConstants.kClimberControlDeadband.get()) * ClimberConstants.kClimberManualSpeed.get();

        return moveClimber(leftStick, rightStick);
    }
}