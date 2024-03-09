// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;
import me.nabdev.oxconfig.sampleClasses.ConfigurableSparkPIDController;

public class MAXSwerveModule {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final SparkPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Values for simulation
    private double m_simDriveEncoderPosition;
    private double m_simDriveEncoderVelocity;
    // In radians
    private double m_simCurrentAngle;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String moduleName) {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 5000);
        m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 5000);
        m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 5000);
        m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 5000);
        m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 5000);
        m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 5000);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
                ModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
                ModuleConstants.kTurningMaxOutput);

        new ConfigurableSparkPIDController(m_drivingPIDController,
                moduleName + "/driving",
                moduleName + " driving");
        new ConfigurableSparkPIDController(m_turningPIDController,
                moduleName + "/turning",
                moduleName + " turning");

        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_chassisAngularOffset = chassisAngularOffset;
        if (RobotBase.isReal()) {
            m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        }

        m_drivingEncoder.setPosition(0);

        if (RobotBase.isSimulation()) {
            // The rev physics sim will handle changing the encoder readings.
            REVPhysicsSim.getInstance().addSparkMax(m_drivingSparkMax, DCMotor.getNEO(1));
            // However, it does NOT support kPosition mode for the turning motor, so we use
            // the m_simCurrentAngle variable to keep track of the angle.
            REVPhysicsSim.getInstance().addSparkMax(m_turningSparkMax, DCMotor.getNeo550(1));
        }
    }

    private void simUpdateDrivePosition(SwerveModuleState state) {
        m_simDriveEncoderVelocity = state.speedMetersPerSecond;
        // Loop time is 20ms, so we convert by dividing by 50.
        m_simDriveEncoderPosition += m_simDriveEncoderVelocity / 50.0;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isSimulation()) {
            return getSimState();
        }
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    private SwerveModuleState getSimState() {
        return new SwerveModuleState(m_simDriveEncoderVelocity,
                new Rotation2d(m_simCurrentAngle - m_chassisAngularOffset));
    }

    /**
     * Get the desired state of the module as commanded by the last call to {@link
     * #setDesiredState(SwerveModuleState)}.
     * 
     * @return The desired state of the module.
     */
    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isSimulation()) {
            return getSimPosition();
        }
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d((m_turningEncoder.getPosition() - m_chassisAngularOffset)));
    }

    private SwerveModulePosition getSimPosition() {
        return new SwerveModulePosition(m_simDriveEncoderPosition,
                new Rotation2d(m_simCurrentAngle - m_chassisAngularOffset));
    }

    private Rotation2d getRotation() {
        if (RobotBase.isSimulation()) {
            return new Rotation2d(m_simCurrentAngle);
        }
        return new Rotation2d(m_turningEncoder.getPosition());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, getRotation());

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
                CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;

        if (RobotBase.isSimulation()) {
            simUpdateDrivePosition(optimizedDesiredState);
            m_simCurrentAngle = optimizedDesiredState.angle.getRadians();
        }
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}