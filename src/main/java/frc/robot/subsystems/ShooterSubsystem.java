package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class ShooterSubsystem extends SubsystemBase {
    /*
     * Defines the motors that shoot the notes
     */
    CANSparkMax topMotor = new CANSparkMax(CANConstants.kShooterTopMotorPort, MotorType.kBrushless);
    CANSparkMax bottomMotor = new CANSparkMax(CANConstants.kShooterBottomMotorPort, MotorType.kBrushless);

    /*
     * Encoders for Shooter wheels
     */
    RelativeEncoder topShooterMoterEncoder = topMotor.getEncoder();
    RelativeEncoder bottomShooterMotorEncoder = bottomMotor.getEncoder();

    double motorRPM = 0;

    public double speakerRPM = 0;
    public double ampRPM = 0;

    boolean isShooterRunning = false;

    public static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private SparkPIDController topPidController;
    private SparkPIDController bottomPidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    double maxVel = 0;
    double maxAccel = 0;
    public static final int kCPR = 8192;

    private RelativeEncoder m_topAlternateEncoder;
    private RelativeEncoder m_bottomAlternateEncoder;

    private double currentTargetRPM = 0;

    public void setShooterRPM(double motorRPM) {
        currentTargetRPM = motorRPM;
    }

    /*
     * Updates the shooters information on what the current angle of the robot is
     */
    @Override
    public void periodic() {

    }

    public void stopShooter() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    public void consumeShooterInput(boolean isTheAButtonPressed) {
        motorRPM = motorRPM * ShooterConstants.kShooterManualSpeed.get();

        if (isTheAButtonPressed) {
            setShooterRPM(motorRPM);
            handlePID();
        } else {
            stopShooter();
        }
    }

    public ShooterSubsystem() {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        m_topAlternateEncoder = topMotor.getAlternateEncoder(kAltEncType, kCPR);
        m_bottomAlternateEncoder = bottomMotor.getAlternateEncoder(kAltEncType, kCPR);

        // PID Coedficients
        kP = 0.1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        topPidController = topMotor.getPIDController();

        bottomPidController = bottomMotor.getPIDController();

        topPidController.setP(kP);
        topPidController.setI(kI);
        topPidController.setD(kD);
        topPidController.setIZone(kIz);
        topPidController.setFF(kFF);
        topPidController.setOutputRange(kMinOutput, kMaxOutput);
        topPidController.setSmartMotionMaxVelocity(maxVel, 0);
        topPidController.setSmartMotionMaxAccel(maxAccel, 0);

        topPidController.setFeedbackDevice(m_topAlternateEncoder);
        // ----------------------------------------------------------------------------------
        bottomPidController.setP(kP);
        bottomPidController.setI(kI);
        bottomPidController.setD(kD);
        bottomPidController.setIZone(kIz);
        bottomPidController.setFF(kFF);
        bottomPidController.setOutputRange(kMinOutput, kMaxOutput);
        bottomPidController.setSmartMotionMaxVelocity(maxVel, 0);
        bottomPidController.setSmartMotionMaxAccel(maxAccel, 0);

        bottomPidController.setFeedbackDevice(m_bottomAlternateEncoder);
    }

    public void handlePID() {
        topPidController.setReference(currentTargetRPM, CANSparkMax.ControlType.kVelocity);
        bottomPidController.setReference(currentTargetRPM, CANSparkMax.ControlType.kVelocity);
    }

    public void speakerSpeed() {
        currentTargetRPM = speakerRPM;
    }

    public double getTopMotorRPM() {
        return topShooterMoterEncoder.getVelocity();
    }

    public double getBottomMotorRPM() {
        return bottomShooterMotorEncoder.getVelocity();
    }

    public boolean shooterAtSpeed() {
        double tolerance = ShooterConstants.kShooterToleranceRPM.get();
        return Math.abs(topShooterMoterEncoder.getVelocity() - currentTargetRPM) < tolerance
                && Math.abs(bottomShooterMotorEncoder.getVelocity() - currentTargetRPM) < tolerance;
    }

    public void ampSpeed() {
        currentTargetRPM = ampRPM;
    }
}
