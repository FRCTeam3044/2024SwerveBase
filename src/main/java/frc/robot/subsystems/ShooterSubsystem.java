package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ShooterSubsystem extends SubsystemBase {
    /*
     * Defines the motors that shoot the notes
     */
    CANSparkMax topMotor = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax bottomMotor = new CANSparkMax(0, MotorType.kBrushless);

    /*
     * Encoders for Shooter wheels
     */
    RelativeEncoder topShooterMoterEncoder = topMotor.getEncoder();
    RelativeEncoder bottomShooterMotorEncoder = bottomMotor.getEncoder();

    /*
     * Change this to change the power of the motors
     */
    double motorRPM = 0;

    /*
     * Change this to change the angles of the shooters
     */
    double shooterAngle = 0;

    boolean isShooterRunning = false;

    private SparkPIDController m_pidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    double maxVel = 0;
    double maxAccel = 0;

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
        // PID Coedficients
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        m_pidController = topMotor.getPIDController();
        m_pidController = bottomMotor.getPIDController();

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        m_pidController.setSmartMotionMaxAccel(maxAccel, 0);
    }
}
