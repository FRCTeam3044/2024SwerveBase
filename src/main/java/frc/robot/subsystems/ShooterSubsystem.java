package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableSparkPIDController;

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

    public ConfigurableParameter<Double> speakerRPM = new ConfigurableParameter<Double>(100.0, "Speaker Shooter RPM");
    public ConfigurableParameter<Double> ampTopRPM = new ConfigurableParameter<Double>(600.0, "Amp Top Shooter RPM");
    public ConfigurableParameter<Double> ampBottomRPM = new ConfigurableParameter<Double>(1600.0,
            "Amp Bottom Shooter RPM");
    public ConfigurableParameter<Double> lobRPM = new ConfigurableParameter<Double>(
            2000.0, "Lob Speed");

    public ConfigurableParameter<Double> shooterSpinupTime = new ConfigurableParameter<Double>(1.25,
            "Shooter Spinup Time");

    public static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private SparkPIDController topPidController;
    private SparkPIDController bottomPidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double maxVel = 5000;
    private ConfigurableParameter<Double> maxAccel = new ConfigurableParameter<Double>(10000.0,
            "Shooter Max Accelleration");
    public static final int kCPR = 8192;

    private double currentBottomTargetRPM = 0;
    private double currentTopTargetRPM = 0;

    private boolean isShooting = false;
    private Timer timeSinceShooting = new Timer();

    public void setShooterRPM(double motorRPM) {
        currentBottomTargetRPM = motorRPM;
        currentTopTargetRPM = motorRPM;
    }

    /*
     * Updates the shooters information on what the current angle of the robot is
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Top Encoder", topMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Bottom Encoder", bottomMotor.getEncoder().getVelocity());
    }

    public void stopShooter() {
        isShooting = false;
        currentBottomTargetRPM = 0;
        currentTopTargetRPM = 0;
        topMotor.set(0);
        bottomMotor.set(0);
    }

    public ShooterSubsystem() {
        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setSmartCurrentLimit(30);
        bottomMotor.setSmartCurrentLimit(30);

        // PID Coedficients
        kP = 0;
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
        topPidController.setSmartMotionMaxAccel(maxAccel.get(), 0);

        bottomPidController.setP(kP);
        bottomPidController.setI(kI);
        bottomPidController.setD(kD);
        bottomPidController.setIZone(kIz);
        bottomPidController.setFF(kFF);
        bottomPidController.setOutputRange(kMinOutput, kMaxOutput);
        bottomPidController.setSmartMotionMaxVelocity(maxVel, 0);
        bottomPidController.setSmartMotionMaxAccel(maxAccel.get(), 0);

        new ConfigurableSparkPIDController(bottomPidController,
                "Shooter bottom pid");
        new ConfigurableSparkPIDController(topPidController,
                "Shooter top pid");
    }

    private void handlePID() {
        if ((currentTopTargetRPM > 10 || currentBottomTargetRPM > 10)) {
            if (!isShooting) {
                isShooting = true;
                timeSinceShooting.reset();
                timeSinceShooting.start();
            }
        } else if (isShooting) {
            isShooting = false;
        }
        topPidController.setReference(currentTopTargetRPM, CANSparkMax.ControlType.kSmartVelocity);
        bottomPidController.setReference(-currentBottomTargetRPM, CANSparkMax.ControlType.kSmartVelocity);
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
        return (isShooting && timeSinceShooting.hasElapsed(shooterSpinupTime.get()));
    }

    private void setTargetSpeed(double topRPM, double bottomRPM) {
        currentTopTargetRPM = topRPM;
        currentBottomTargetRPM = bottomRPM;
    }

    public void saveShotData() {
        double dist = RobotContainer.m_robotDrive.getPose().getTranslation()
                .getDistance(AutoTargetUtils.getShootingTarget().getTranslation());
        ChassisSpeeds speeds = RobotContainer.m_robotDrive.getChassisSpeeds();

        RobotContainer.m_autoAiming.addData(dist, RobotContainer.elevator.getAngle(), getTopMotorRPM(),
                getBottomMotorRPM(), speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Command shoot(double topRPM, double bottomRPM) {
        return Commands.runEnd(() -> {
            setTargetSpeed(topRPM, bottomRPM);
            handlePID();
        }, this::stopShooter, this).withName("Shooter");
    }

    public Command shoot(double rpm) {
        return shoot(rpm, rpm);
    }

    public Command speaker() {
        return shoot(speakerRPM.get()).withName("Speaker Shooter");
    }

    public Command amp() {
        return shoot(ampTopRPM.get(), ampBottomRPM.get()).withName("Amp Shooter");
    }

    public Command lob() {
        return shoot(lobRPM.get()).withName("Lob Shooter");
    }

    public Command shootPercentage(double top, double bottom) {
        return Commands.runEnd(() -> {
            topMotor.set(top);
            bottomMotor.set(bottom);
        }, this::stopShooter, this).withName("Shoot Percentage");
    }

    public Command shootPercentage(double percentage) {
        return shootPercentage(percentage, -percentage);
    }

    public Command slow() {
        double speed = ShooterConstants.kShooterManualSlowSpeed.get();
        return shootPercentage(speed).withName("Slow Shooter (Percentage)");
    }

    public Command shoot() {
        double speed = ShooterConstants.kShooterManualSpeed.get();
        return shootPercentage(speed).withName("Normal Shooter (Percentage)");
    }

    public Command stop() {
        return Commands.runOnce(this::stopShooter, this).withName("Stop Shooter");
    }
}
