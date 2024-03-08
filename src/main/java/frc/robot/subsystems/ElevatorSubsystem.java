package frc.robot.subsystems;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.function.Consumer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;
import frc.robot.utils.USBLocator;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableSparkPIDController;

public class ElevatorSubsystem extends SubsystemBase {

    private ArrayList<Double> elevatorAngleCalibration = new ArrayList<Double>();
    private ArrayList<Double> elevatorEncoderCalibration = new ArrayList<Double>();
    public boolean calibrationModeEnabled = false;

    public RobotContainer m_robotContainer;

    public CANSparkMax elevatorMotorOne = new CANSparkMax(CANConstants.kElevatorMotorOnePort, MotorType.kBrushless);
    public CANSparkMax elevatorMotorTwo = new CANSparkMax(CANConstants.kElevatorMotorTwoPort, MotorType.kBrushless);
    // DigitalInput elevatorTopLimitSwitch = new
    // DigitalInput(CANConstants.kElevatorTopLimitSwitch);

    // DigitalInput elevatorBottomLimitSwitch = new
    // DigitalInput(CANConstants.kElevatorBottomLimitSwitch);

    // public AbsoluteEncoder elevatorPivotEncoder =
    // elevatorMotorOne.getAbsoluteEncoder(Type.kDutyCycle);
    public DutyCycleEncoder elevatorPivotEncoder = new DutyCycleEncoder(CANConstants.kElevatorPivotEncoderPort);

    private SparkPIDController pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    double maxVel = 0;
    double maxAccel = 0;
    double minVel = 0;

    RelativeEncoder motorOneEncoder = elevatorMotorOne.getEncoder();
    RelativeEncoder motorTwoEncoder = elevatorMotorTwo.getEncoder();
    double currentTargetRotations = 0;
    private boolean hasInitialized = false;

    public ElevatorSubsystem() {
        elevatorMotorOne.restoreFactoryDefaults();
        elevatorMotorTwo.restoreFactoryDefaults();

        elevatorMotorOne.setIdleMode(IdleMode.kBrake);
        elevatorMotorTwo.setIdleMode(IdleMode.kBrake);

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
        pidController.setSmartMotionMinOutputVelocity(minVel, 0);

        new ConfigurableSparkPIDController(pidController,
                "Elevator Angle PID");
        Consumer<Double> setVel = (Double v) -> pidController.setSmartMotionMaxVelocity(v, 0);
        new ConfigurableParameter<Double>(maxVel, "Elevator SmartMotion Max Velocity", setVel);
        Consumer<Double> setAccel = (Double a) -> pidController.setSmartMotionMaxAccel(a, 0);
        new ConfigurableParameter<Double>(maxAccel, "Elevator SmartMotion Max Accel", setAccel);
        Consumer<Double> setMinVel = (Double m) -> pidController.setSmartMotionMinOutputVelocity(m, 0);
        new ConfigurableParameter<Double>(minVel, "Elevator SmartMotion Min Velocity", setMinVel);

        new ConfigurableSparkPIDController(pidController,
                "Elevator Angle PID");

        elevatorMotorTwo.follow(elevatorMotorOne);
    }

    // Sets the intake, shooter, and transit to the postion that we want it to be in
    public void moveElevator(double motorSpeed) {
        elevatorMotorOne.set(motorSpeed);
    }

    // Shifts the intake, shooter, and transit to the default postion that makes it
    // easier pick up notes
    public void setToIntakeMode() {
        currentTargetRotations = angleToRotations(ElevatorConstants.kIntakeAngle.get());
    }

    // Shifts the intake, shooter, and transit to the position used for shooting
    // into an amp
    public void setToAmpMode() {
        currentTargetRotations = angleToRotations(ElevatorConstants.kAmpAngle.get());
    }

    public void setToSubwooferMode() {
        currentTargetRotations = angleToRotations(ElevatorConstants.kSubwooferAngle.get());
    }

    public void setAngle(double setAngle) {
        currentTargetRotations = angleToRotations(setAngle);
    }

    public double getAngle() {
        // return elevatorPivotEncoder.getPosition();
        return elevatorPivotEncoder.getAbsolutePosition();
    }

    public void pidHandler() {
        pidController.setReference(currentTargetRotations, CANSparkMax.ControlType.kSmartMotion);
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

    private double angleToRotations(double angle) {
        double raw = -89.2 + (317 * angle) + (-183 * Math.pow(angle, 2));
        return MathUtil.clamp(raw, 0.0, 21);
    }

    @Override
    public void periodic() {
        if (!hasInitialized) {
            SmartDashboard.putNumber("Arm/Starting absolute pos", getAngle());
            motorOneEncoder.setPosition(angleToRotations(getAngle()));
            hasInitialized = true;
        }
        SmartDashboard.putNumber("Arm/MotorOnePos", motorOneEncoder.getPosition());
        SmartDashboard.putNumber("Arm/MotorTwoPos", motorTwoEncoder.getPosition());
        SmartDashboard.putNumber("Arm/TargetPos", currentTargetRotations);
        SmartDashboard.putNumber("Arm/CurrentAngle", getAngle());
        SmartDashboard.putNumber("Arm/PredictedEncoderForAngle", angleToRotations(getAngle()));
        SmartDashboard.putNumber("Arm/MotorOneVelocity", motorOneEncoder.getVelocity());

        if (calibrationModeEnabled) {
            elevatorAngleCalibration.add(getAngle());
            elevatorEncoderCalibration.add(motorOneEncoder.getPosition());
            if (RobotContainer.m_driverController.getHID().getPOV() == 90) {
                String path = RobotBase.isReal() ? USBLocator.getUSBPath() + "/calibration.csv"
                        : Filesystem.getDeployDirectory() + "/calibration.csv";
                try (PrintWriter writer = new PrintWriter(
                        new FileWriter(path))) {
                    writer.println("Angle,Encoder");
                    for (int i = 0; i < elevatorAngleCalibration.size(); i++) {
                        double angle = elevatorAngleCalibration.get(i);
                        double encoder = elevatorEncoderCalibration.get(i);
                        writer.println(angle + "," + encoder);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}