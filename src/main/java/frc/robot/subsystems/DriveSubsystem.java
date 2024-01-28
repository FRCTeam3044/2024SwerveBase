// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      "frontLeftModule");

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      "frontRightModule");

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      "rearLeftModule");

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      "rearRightModule");

  // The gyro sensor (NavX)
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  public final Field2d field = new Field2d();

  public final Pathfinder pathfinder;

  private final SwerveDrivePoseEstimator poseEstimator;

  // Sim values
  private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  private SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  // In radians
  private double m_simYaw;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getGyroAngleDegrees()),
      getModulePositions());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Define the standard deviations for the pose estimator, which determine how
    // fast the pose
    // estimate converges to the vision measurement. This should depend on the
    // vision measurement
    // noise
    // and how many or how frequently vision measurements are applied to the pose
    // estimator.

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getGyroAngleDegrees()),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    pathfinder = new PathfinderBuilder(Field.CRESCENDO_2024)
        .setInjectPoints(true)
        .setPointSpacing(0.25)
        .setRobotLength(DriveConstants.kWheelBase)
        .setRobotWidth(DriveConstants.kTrackWidth)
        .build();

    // ArrayList<Edge> edges = m_pathfinder.visualizeEdges();
    // PathfindingDebugUtils.drawLines("Field Map", edges,
    // m_pathfinder.visualizeVertices());
    // PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
    // m_pathfinder.visualizeInflatedVertices());
  }

  private double getGyroAngleDegrees() {
    double gyroRadians = Math.toRadians(-m_gyro.getAngle());
    return Math.toDegrees(MathUtil.angleModulus(gyroRadians +
        Math.PI / 2));
  }

  @Override
  public void periodic() {
    // Update the pose estimator in the periodic block
    poseEstimator.update(Rotation2d.fromDegrees(getGyroAngleDegrees()), getModulePositions());
    Logger.recordOutput("gyro", getGyroAngleDegrees());
    Logger.recordOutput("gyroRad", Math.toRadians(getGyroAngleDegrees()));
    Logger.recordOutput("ModuleStatesMeasured", getModuleStates());
    Logger.recordOutput("ModuleStatesDesired", getDesiredModuleStates());
    field.setRobotPose(getPose());

    SmartDashboard.putData("Field", field);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the module states of the swerve drive.
   * 
   * @return An array of SwerveModuleStates. Index 0 is front left, index 1 is
   *         front right, index 2 is rear left, and index 3 is rear right.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /**
   * Returns the desired module states of the swerve drive.
   * 
   * @return An array of SwerveModuleStates. Index 0 is front left, index 1 is
   *         front right, index 2 is rear left, and index 3 is rear right.
   */
  public SwerveModuleState[] getDesiredModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getDesiredState(),
        m_frontRight.getDesiredState(),
        m_rearLeft.getDesiredState(),
        m_rearRight.getDesiredState()
    };
  }

  /**
   * Returns the module positions of the swerve drive.
   * 
   * @return An array of SwerveModuleStates. Index 0 is front left, index 1 is
   *         front right, index 2 is rear left, and index 3 is rear right.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroAngleDegrees()),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate.get() / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond.get();
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond.get();
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed.get();

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroAngleDegrees()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond.get());
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond.get());
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Robot Rot (Rad)", getPose().getRotation().getRadians());

    ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
    angle.set(Math.toDegrees(m_simYaw));

    REVPhysicsSim.getInstance().run();
  }

}