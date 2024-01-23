package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;

public class ClimberSubsystem {
    TalonSRX leftClimberMotor = new TalonSRX(0);
    TalonSRX rightClimberMotor = new TalonSRX(0);
    Encoder leftClimberEncoder = new Encoder(null, null);
    Encoder rightClimberEncoder = new Encoder(null, null);

    // varible for determining whether or not the arm is up or down
    int armposition = 0;

    // power going into each of the motors
    int leftMotorPower = 0;
    int rightMotorPower = 0;

    // raises the arm
    public void raiseArm() {

    }

    // lowers the arm
    public void lowerArm() {

    }
}