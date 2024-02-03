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

    DigitalInput elevatorTopLimitSwitch = new DigitalInput(0);

    DigitalInput elevatorBottomLimitSwitch = new DigitalInput(0);

    PIDController pid = new PIDController(0, 0, 0);

    // The angle for shooting in the amp (currently set to 0)
    double ampAngle = 0;

    // Sets the intake, shooter, and transit to the postion that we want it to be in
    public void moveElevator(double motorSpeed) {
        elevatorMotorOne.set(motorSpeed));
        elevatorMotorTwo.set(motorSpeed));
    }

    // Shifts the intake, shooter, and transit to the default postion that makes it
    // easier pick up notes
    public void setToIntakeMode() {

    }

    // Shifts the intake, shooter, and transit to the position used for shooting
    // into an amp
    public void setToAmpMode() {

    }

    // Tells us when the elevator has hit the top of it's height
    public void readTopLimitSwitch() {

    }

    // Tells us when the elevator has hit the bottom of it's height
    public void readBottomLimitSwitch() {

    }

    public void consumeElevatorInput(double elevatorSpeed) {
        moveElevator(elevatorSpeed);
    }
}
