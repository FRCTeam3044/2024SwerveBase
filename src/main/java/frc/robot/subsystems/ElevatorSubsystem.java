package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;

public class ElevatorSubsystem extends SubsystemBase {

    public RobotContainer m_robotContainer;

    public CANSparkMax elevatorMotorOne = new CANSparkMax(CANConstants.kElevatorMotorOnePort, MotorType.kBrushless);
    public CANSparkMax elevatorMotorTwo = new CANSparkMax(CANConstants.kElevatorMotorTwoPort, MotorType.kBrushless);

    AbsoluteEncoder shooterEncoderOne = elevatorMotorOne.getAbsoluteEncoder(Type.kDutyCycle);

    DigitalInput elevatorTopLimitSwitch = new DigitalInput(CANConstants.kElevatorTopLimitSwitch);

    DigitalInput elevatorBottomLimitSwitch = new DigitalInput(CANConstants.kElevatorBottomLimitSwitch);

    private SparkPIDController pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    double maxVel = 0;
    double maxAccel = 0;

    PIDController pid = new ConfigurablePIDController(0, 0, 0, "Elevator PID");

    // The angle for shooting in the amp (currently set to 0)
    public double ampAngle = 0;
    public double intakeAngle = 0;

    RelativeEncoder motorOneEncoder = elevatorMotorOne.getEncoder();
    RelativeEncoder motorTwoEncoder = elevatorMotorTwo.getEncoder();
    double currentTargetRotations = 0;

    public ElevatorSubsystem() {
        elevatorMotorOne.restoreFactoryDefaults();
        elevatorMotorTwo.restoreFactoryDefaults();

        // PID coefficients
        kP = 0.1;
        kI = 0;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.1;
        kMinOutput = -0.1;

        pidController = elevatorMotorOne.getPIDController();

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidController.setSmartMotionMaxVelocity(maxVel, 0);
        pidController.setSmartMotionMaxAccel(maxAccel, 0);

        elevatorMotorOne.follow(elevatorMotorTwo);
    }

    // Sets the intake, shooter, and transit to the postion that we want it to be in
    public void moveElevator(double motorSpeed) {
        elevatorMotorOne.set(motorSpeed);
        elevatorMotorTwo.set(motorSpeed);
    }

    // Shifts the intake, shooter, and transit to the default postion that makes it
    // easier pick up notes
    public void setToIntakeMode() {
        currentTargetRotations = intakeAngle;
    }

    // Shifts the intake, shooter, and transit to the position used for shooting
    // into an amp
    public void setToAmpMode() {
        currentTargetRotations = ampAngle;
    }

    public void setAngle(double setAngle) {
        currentTargetRotations = setAngle;
    }

    /**
     * Read the top limit switch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean readTopLimitSwitch() {
        return !elevatorTopLimitSwitch.get();
    }

    /**
     * Read the bottom limit switch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean readBottomLimitSwitch() {
        return !elevatorBottomLimitSwitch.get();
    }

    public double getAngle() {
        double currentAngle = shooterEncoderOne.getPosition();
        return currentAngle;
    }

    public void pidHandler() {
        pidController.setReference(currentTargetRotations, CANSparkMax.ControlType.kPosition);
    }

    public void consumeElevatorInput(double leftStickY) {
        if (Math.abs(leftStickY) > 0.1) {
            elevatorMotorOne.set(leftStickY * Math.abs(leftStickY));
        } else {
            elevatorMotorOne.set(0);
        }
    }

    public boolean elevatorAtAngle() {
        double tolerance = ElevatorConstants.kElevatorTolerance.get();
        return Math.abs(motorOneEncoder.getPosition() - currentTargetRotations) < tolerance
                && Math.abs(motorTwoEncoder.getPosition() - currentTargetRotations) < tolerance;
    }
}
