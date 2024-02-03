package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem {
    CANSparkMax elevatorMotorOne = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax elevatorMotorTwo = new CANSparkMax(0, MotorType.kBrushless);

    AbsoluteEncoder shooterEncoderOne = elevatorMotorOne.getAbsoluteEncoder(Type.kDutyCycle);

    DigitalInput elevatorTopLimitSwitch = new DigitalInput(0);

    DigitalInput elevatorBottomLimitSwitch = new DigitalInput(0);

    private SparkPIDController m_pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    double maxVel = 0;
    double maxAccel = 0;

    PIDController pid = new PIDController(0, 0, 0);

    // The angle for shooting in the amp (currently set to 0)
    double ampAngle = 0;

    public ElevatorSubsystem() {
        // PID coefficients
        kP = 0.1;
        kI = 0;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.1;
        kMinOutput = -0.1;

        m_pidController = elevatorMotorOne.getPIDController();

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        m_pidController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    // Sets the intake, shooter, and transit to the postion that we want it to be in
    public void moveElevator(double motorSpeed) {
        elevatorMotorOne.set(motorSpeed);
        elevatorMotorTwo.set(motorSpeed);
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

    public void pidHandler(double meters) {
        double rotations = meters; // needs math to convert meters to rotation
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
}
