package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;

public class ClimberSubsystem {
    TalonSRX leftClimberMotor = new TalonSRX(0);
    TalonSRX rightClimberMotor = new TalonSRX(0);
    Encoder leftClimberEncoder = new Encoder(null, null);
    Encoder rightClimberEncoder = new Encoder(null, null);

    // power going into each of the motors
    int leftMotorPower = 0;
    int rightMotorPower = 0;

    // raises the arm
    public void raiseArms() {

    }

    // lowers the arm
    public void lowerArms() {
        //
    }
}