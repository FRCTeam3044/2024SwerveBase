package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem {
    CANSparkMax elevatorMotorOne = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax elevatorMotorTwo = new CANSparkMax(0, MotorType.kBrushless);
    
    AbsoluteEncoder shooterEncoderOne = elevatorMotorOne.getAbsoluteEncoder(Type.kDutyCycle);
    AbsoluteEncoder shooterEncoderTwo = elevatorMotorTwo.getAbsoluteEncoder(Type.kDutyCycle);

    // Could cange depending on which switch is used
    DigitalInput elevatorLimitSwitch = new DigitalInput(0);

    PIDController pid = new PIDController(0, 0, 0);

    public void setAngleAdjustment() {

    }

    public void setToIntakeMode() {

    }

    public void setToShooterMode() {
        
    }
}
