package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

public class ElevatorSubsystem {
    CANSparkMax elevatorMotor = new CANSparkMax();
    
    AbsoluteEncoder shooterEncoder = new AbsoluteEncoder();

    PIDController pid = new PIDController(0, 0, 0);

    public void setAngleAdjustment() {

    }

    public void setToIntakeMode() {

    }
}
