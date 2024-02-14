package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

    public double speakerSpeedPercentage = 0;
    public double ampSpeedPercentage = 0;

    boolean isShooterRunning = false;

    private SparkPIDController pidController;
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
        topMotor.set(1);
        bottomMotor.set(-1);
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
        kP = 0.1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        pidController = topMotor.getPIDController();

        pidController = topMotor.getPIDController();
        pidController = bottomMotor.getPIDController();

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidController.setSmartMotionMaxVelocity(maxVel, 0);
        pidController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    public void speakerPidHandler() {
        double rotations = speakerSpeedPercentage;
        pidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);
    }

    public void ampPidHandler() {
        double rotations = ampSpeedPercentage;
        pidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);
    }
}

