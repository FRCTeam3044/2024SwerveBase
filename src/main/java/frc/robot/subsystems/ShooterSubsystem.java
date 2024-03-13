package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.oxconfig.ConfigurableParameter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

public class ShooterSubsystem extends SubsystemBase {
    /*
     * Defines the motors that shoot the note
     */
    private CANSparkMax topMotor = new CANSparkMax(CANConstants.kShooterTopMotorPort, MotorType.kBrushless);
    private CANSparkMax bottomMotor = new CANSparkMax(CANConstants.kShooterBottomMotorPort, MotorType.kBrushless);

    /*
     * Encoders for Shooter wheels
     */
    private RelativeEncoder topShooterMoterEncoder = topMotor.getAlternateEncoder(8192);
    private RelativeEncoder bottomShooterMotorEncoder = bottomMotor.getAlternateEncoder(8192);

    // TODO: Get values
    public ConfigurableParameter<Double> speakerRPM = new ConfigurableParameter<Double>(100.0, "Speaker Shooter RPM");
    public double ampRPM = 0;

    public ConfigurableParameter<Double> shooterSpinupTime = new ConfigurableParameter<Double>(1.0,
            "Shooter Spinup Time");

    public static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private SparkPIDController topPidController;
    private SparkPIDController bottomPidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double maxVel = 0;
    private double maxAccel = 0;
    public static final int kCPR = 8192;

    private RelativeEncoder m_topAlternateEncoder;
    private RelativeEncoder m_bottomAlternateEncoder;

    private double currentTargetRPM = 0;

    private boolean isShooting = false;
    private Timer timeSinceShooting = new Timer();

    public void setShooterRPM(double motorRPM) {
        currentTargetRPM = motorRPM;
    }

    /*
     * Updates the shooters information on what the current angle of the robot is
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Top Encoder", m_topAlternateEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Bottom Encoder", m_bottomAlternateEncoder.getVelocity());
    }

    public void stopShooter() {
        isShooting = false;
        currentTargetRPM = 0;
        topMotor.set(0);
        bottomMotor.set(0);
    }

    public void consumeShooterInput(boolean shoot, boolean slow) {
        double output = slow ? ShooterConstants.kShooterManualSlowSpeed.get()
                : ShooterConstants.kShooterManualSpeed.get();
        if (shoot) {
            if (!isShooting) {
                isShooting = true;
                timeSinceShooting.reset();
                timeSinceShooting.start();
            }
            topMotor.set(output);
            bottomMotor.set(-output);
        } else {
            stopShooter();
        }

    }

    public ShooterSubsystem() {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setSmartCurrentLimit(30);
        bottomMotor.setSmartCurrentLimit(30);

        // bottomMotor.setInverted(false);

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
        currentTargetRPM = speakerRPM.get();
    }

    public double getTopMotorRPM() {
        return topShooterMoterEncoder.getVelocity();
    }

    public double getBottomMotorRPM() {
        return bottomShooterMotorEncoder.getVelocity();
    }

    public boolean shooterAtSpeed() {
        // double tolerance = ShooterConstants.kShooterToleranceRPM.get();
        // return Math.abs(topShooterMoterEncoder.getVelocity() - currentTargetRPM) <
        // tolerance
        // && Math.abs(bottomShooterMotorEncoder.getVelocity() - currentTargetRPM) <
        // tolerance;
        return (isShooting && timeSinceShooting.get() > shooterSpinupTime.get());
    }

    public void ampSpeed() {
        currentTargetRPM = ampRPM;
    }

    public void saveShotData() {
        double dist = RobotContainer.m_robotDrive.getPose().getTranslation()
                .getDistance(AutoTargetUtils.getShootingTarget().getTranslation());
        ChassisSpeeds speeds = RobotContainer.m_robotDrive.getChassisSpeeds();

        RobotContainer.m_autoAiming.addData(dist, RobotContainer.elevator.getAngle(), getTopMotorRPM(),
                getBottomMotorRPM(), speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
}
