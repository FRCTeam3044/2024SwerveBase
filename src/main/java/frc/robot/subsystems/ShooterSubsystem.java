package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

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

    public void setShooterRPM(double motorRPM) {

    }

    /*
     * Updates the shooters information on what the current angle of the robot is
     */
    public void updatePeriodic() {

    }

    private void runShooter() {
        topMotor.set(motorRPM);
        bottomMotor.set(-motorRPM);
    }

    private void stopShooter() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    public void consumeShooterInput(boolean isTheAButtonPressed) {
        if (isTheAButtonPressed) {
            runShooter();
        } else {
            stopShooter();
        }
    }

    public ShooterSubsystem() {

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

    public void topSpeakerPidHandler(double speed) {
        double rotations = speed;
        topPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void topAmpPidHandler(double speed) {
        double rotations = speed;
        topPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void bottomSpeakerPidHandler(double speed) {
        double rotations = speed;
        bottomPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public void bottomAmpPidHandler(double speed) {
        double rotations = speed;
        bottomPidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
}
