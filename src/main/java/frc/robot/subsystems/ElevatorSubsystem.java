package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorSubsystem {
    CANSparkMax elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    
}
