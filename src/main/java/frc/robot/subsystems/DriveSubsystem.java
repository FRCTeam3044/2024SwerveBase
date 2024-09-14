// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.MemoizedSupplier;
import frc.robot.utils.PathfindingDebugUtils;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.TargetRotationController;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.structures.Vector;
import me.nabdev.pathfinding.structures.Vertex;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    // private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    public final Field2d field = new Field2d();

    private final Pathfinder pathfinder;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final ConfigurableParameter<Double> slowPercent = new ConfigurableParameter<Double>(0.7,
            "Slow Speed Percent");

    public DoubleSupplier allianceInput = () -> {
        if (RobotBase.isSimulation())
            return 1.0;
        return (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue) ? -1.0 : 1.0;
    };

    // Sim values
    private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    private SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // In radians
    private double m_simYaw;

    private LinearFilter distanceToShootingTargetFilter = LinearFilter.movingAverage(20);
    public double distanceToShootingTarget;

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
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(getGyroAngleDegrees()),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        pathfinder = new PathfinderBuilder(
                Field.CRESCENDO_2024)
                .setInjectPoints(true)
                .setPointSpacing(0.5)
                .setCornerPointSpacing(0.05)
                .setRobotLength(DriveConstants.kRobotSize)
                .setRobotWidth(DriveConstants.kRobotSize)
                .setCornerDist(0.3)
                .setCornerCutDist(0.01)
                .build();

        ArrayList<Edge> edges = pathfinder.visualizeEdges();
        PathfindingDebugUtils.drawLines("Field Map", edges,
                pathfinder.visualizeVertices());
        PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
                pathfinder.visualizeInflatedVertices());
    }

    private double getGyroAngleDegrees() {
        // double gyroRadians = Math.toRadians(-m_gyro.getAngle());
        double gyroRadians = Math.toRadians(-m_gyro.getAngle() - Math.PI / 2);
        double wrapped = Math.toDegrees(MathUtil.angleModulus(gyroRadians + Math.PI /
                2));
        return wrapped;
    }

    @Override
    public void periodic() {
        pathfinder.periodic();
        // Update the pose estimator in the periodic block
        poseEstimator.update(Rotation2d.fromDegrees(getGyroAngleDegrees()), getModulePositions());
        Logger.recordOutput("ModuleState", getModuleStates());
        // Logger.recordOutput("ModuleStatesDesired", getDesiredModuleStates());
        field.setRobotPose(getPose());

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Gyro Raw", getGyroAngleDegrees());
        Pose2d shootingTarget = AutoTargetUtils.getShootingTarget();
        if (shootingTarget != null) {
            Translation2d shootingTargetT = shootingTarget.getTranslation();
            distanceToShootingTarget = distanceToShootingTargetFilter
                    .calculate(shootingTargetT.getDistance(getPose().getTranslation()));
            SmartDashboard.putNumber("Dist To Speaker", distanceToShootingTarget);
        }

        // ArrayList<Vertex> inflatedVertices = pathfinder.visualizeInflatedVertices();
        // PathfindingDebugUtils.drawLines("Visibility Graph",
        // pathfinder.visualizeNeighbors(), inflatedVertices);
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
    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, false, false);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed           Speed of the robot in the x direction (forward).
     * @param ySpeed           Speed of the robot in the y direction (sideways).
     * @param rot              Angular rate of the robot.
     * @param fieldRelative    Whether the provided x and y speeds are relative to
     *                         the
     *                         field.
     * @param rateLimit        Whether to enable rate limiting for smoother control.
     * @param absoluteRotSpeed If true, rot is interpreted directly as omega radians
     *                         per second.
     */
    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit,
            boolean absoluteRotSpeed, boolean slow) {
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
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
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
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond.get()
                * (slow ? slowPercent.get() : 1);
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond.get()
                * (slow ? slowPercent.get() : 1);
        double rotDelivered = absoluteRotSpeed ? rot : rotSpeedFromJoystick(rot, rateLimit);
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        Rotation2d.fromDegrees(getPose().getRotation().getDegrees()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        setDesiredChassisSpeeds(chassisSpeeds);
    }

    public double rotSpeedFromJoystick(double joystick, boolean rateLimit) {
        if (rateLimit) {
            m_currentRotation = m_rotLimiter.calculate(joystick);
        } else {
            m_currentRotation = joystick;
        }
        return m_currentRotation * DriveConstants.kMaxAngularSpeed.get();
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    private void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the module states to attempt to get the robot to the specified speeds.
     * 
     * @param speeds The desired chassis speeds, ROBOT RELATIVE.
     */
    private void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond.get());
        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
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

    /**
     * Returns the currently estimated pose of the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Trajectory generateTrajectory(ArrayList<Pose2d> waypoints) throws ImpossiblePathException {
        Path path = pathfinder.generatePath(getPose(), waypoints);
        return path.asTrajectory(getTrajectoryConfig(path));
    }

    public Trajectory generateTrajectory(Pose2d start, ArrayList<Pose2d> waypoints) throws ImpossiblePathException {
        Path path = pathfinder.generatePath(getPose(), waypoints);
        return path.asTrajectory(getTrajectoryConfig(path));
    }

    public Trajectory generateTrajectory(Pose2d end) throws ImpossiblePathException {
        Path path = pathfinder.generatePath(getPose(), end);
        return path.asTrajectory(getTrajectoryConfig(path));
    }

    public Trajectory generateTrajectory(Pose2d start, Pose2d end) throws ImpossiblePathException {
        Path path = pathfinder.generatePath(start, end);
        return path.asTrajectory(getTrajectoryConfig(path));
    }

    public Trajectory generateTrajectoryNoAvoidance(Pose2d start, Pose2d end) {
        // TrajectoryConfig config = new
        // TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
        // PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());
        // return pathfinder.generateTrajectory(start, end, config);
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

        Vertex startV = new Vertex(start);
        Vertex endV = new Vertex(end);
        Vector vec = startV.createVectorTo(endV);
        double angle = Math.atan2(vec.y, vec.x);
        Pose2d startTarget = new Pose2d(start.getX(), start.getY(),
                Rotation2d.fromRadians(angle));
        Pose2d endTarget = new Pose2d(end.getX(), end.getY(),
                Rotation2d.fromRadians(angle));
        waypoints.add(startTarget);
        waypoints.add(endTarget);
        TrajectoryConfig config = getTrajectoryConfig(end);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(waypoints, config);
        return traj;
    }

    public TrajectoryConfig getTrajectoryConfig(Path path) {
        TrajectoryConfig config = new TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());

        // TODO: This is wrong, it stutters still on path change
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
        Vector velocity = new Vector(chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond);
        Vertex start = path.getStart();
        Vertex nextWaypoint = path.size() > 0 ? path.get(0) : path.getTarget();
        Vector pathDir = start.createVectorTo(nextWaypoint).normalize();
        double speed = velocity.dotProduct(pathDir);
        config.setStartVelocity(speed);
        // config.setKinematics(DriveConstants.kDriveKinematics);
        // config.setStartVelocity(10);
        return config;
    }

    public TrajectoryConfig getTrajectoryConfig(Pose2d target) {
        TrajectoryConfig config = new TrajectoryConfig(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get());

        // TODO: This is wrong, it stutters still on path change
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
        Vector velocity = new Vector(chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond);
        Vector pathDir = new Vertex(getPose()).createVectorTo(new Vertex(target)).normalize();
        double speed = velocity.dotProduct(pathDir);
        config.setStartVelocity(speed);
        // config.setKinematics(DriveConstants.kDriveKinematics);
        // config.setStartVelocity(10);
        return config;
    }

    // Maybe these should be moved to a utility class somewhere?
    private double sqInput(double input) {
        return Math.copySign(input * input, input);
    }

    private DoubleSupplier joystickToSpeed(DoubleSupplier joystick, boolean alliance) {
        return () -> sqInput(MathUtil.applyDeadband(joystick.getAsDouble(), OIConstants.kDriveDeadband.get()))
                * (alliance ? allianceInput.getAsDouble() : 1);
    }

    private DoubleSupplier negate(DoubleSupplier supplier) {
        return () -> -supplier.getAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        m_simYaw -= chassisSpeed.omegaRadiansPerSecond * 0.02;
        angle.set(Math.toDegrees(m_simYaw));

        REVPhysicsSim.getInstance().run();
    }

    public Command setXMode() {
        return Commands.run(this::setX, this).withName("X Mode");
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative,
            boolean rateLimit, boolean absoluteRotSpeed, BooleanSupplier slow) {
        return Commands.run(() -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative,
                rateLimit, absoluteRotSpeed, slow.getAsBoolean()), this).withName("Drive");
    }

    public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative,
            boolean rateLimit) {
        return Commands.run(() -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative,
                rateLimit), this).withName("Drive");
    }

    public Command manualDrive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX,
            BooleanSupplier slow) {
        DoubleSupplier inputX = joystickToSpeed(leftX, true);
        DoubleSupplier inputY = joystickToSpeed(leftY, true);
        DoubleSupplier inputRot = negate(joystickToSpeed(rightX, true));

        if (RobotBase.isSimulation()) {
            return drive(inputX, negate(inputY), inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), false, slow).withName("Manual Drive");
        } else {
            return drive(inputY, inputX, inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), false, slow).withName("Manual Drive");
        }
    }

    public Command test(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX,
            BooleanSupplier cancelInput) {
        DoubleSupplier inputX = negate(joystickToSpeed(leftX, false));
        DoubleSupplier inputY = negate(joystickToSpeed(leftY, false));
        DoubleSupplier inputRot = () -> cancelInput.getAsBoolean() ? 0
                : negate(joystickToSpeed(rightX, false)).getAsDouble();

        return drive(inputY, inputX, inputRot, false, DriveConstants.kRateLimit.get());
    }

    public Command trackPoint(DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Pose2d> target, boolean flipped) {
        TargetRotationController controller = new TargetRotationController(target, flipped);

        DoubleSupplier inputRot = () -> controller.calculate(getPose(), getChassisSpeeds());
        return drive(xSpeed, ySpeed, inputRot, DriveConstants.kFieldRelative.get(), DriveConstants.kRateLimit.get(),
                true, () -> false).withName("Track Point");
    }

    public Command trackPoint(DoubleSupplier xSpeed, DoubleSupplier ySpeed, Pose2d target, boolean flipped) {
        return trackPoint(xSpeed, ySpeed, () -> target, flipped);
    }

    public Command trackPoint(Supplier<Pose2d> target, boolean flipped) {
        return trackPoint(() -> 0, () -> 0, target, flipped);
    }

    public Command trackPoint(Pose2d target, boolean flipped) {
        return trackPoint(() -> 0, () -> 0, target, flipped);
    }

    public Command driveAndTrackPoint(DoubleSupplier leftX, DoubleSupplier leftY, Supplier<Pose2d> target,
            boolean flipped) {
        DoubleSupplier inputX = joystickToSpeed(leftX, true);
        DoubleSupplier inputY = joystickToSpeed(leftY, true);

        if (RobotBase.isSimulation()) {
            return trackPoint(inputX, negate(inputY), target, flipped).withName("Drive and Track Point");
        } else {
            return trackPoint(inputY, inputX, target, flipped).withName("Drive and Track Point");
        }
    }

    public Command driveAndTrackPoint(DoubleSupplier leftX, DoubleSupplier leftY, Pose2d target, boolean flipped) {
        return driveAndTrackPoint(leftX, leftY, () -> target, flipped);
    }

    private Command followTrajectory(Function<Pose2d, Trajectory> dynamicTrajectory,
            MemoizedSupplier<Trajectory> trajectory, boolean useDynamic, Supplier<Rotation2d> desiredRotation,
            DoubleSupplier desiredRotationSpeed, boolean useRotSpeed) {
        return new Command() {
            private Timer m_timer = new Timer();
            private boolean failed = false;
            private Pose2d startPose;
            private HolonomicDriveController m_controller = PathfindingConstants.getDriveController();

            {
                addRequirements(DriveSubsystem.this);
                setName("Follow Trajectory");
            }

            public void initialize() {
                failed = false;
                m_timer.restart();
                startPose = getPose();
                if (!useDynamic) {
                    trajectory.clear();
                }
            }

            public void execute() {
                Trajectory traj = getTrajectory();
                if (traj == null) {
                    failed = true;
                    return;
                }
                field.getObject("Path").setTrajectory(traj);
                double curTime = m_timer.get();
                State desiredState = traj.sample(curTime);
                ChassisSpeeds targetChassisSpeeds = m_controller.calculate(getPose(), desiredState,
                        desiredRotation.get());
                if (useRotSpeed) {
                    targetChassisSpeeds.omegaRadiansPerSecond = desiredRotationSpeed.getAsDouble();
                }
                setDesiredChassisSpeeds(targetChassisSpeeds);
            }

            public boolean isFinished() {
                if (failed) {
                    return true;
                }
                Trajectory traj = getTrajectory();
                if (traj == null) {
                    failed = true;
                    return true;
                }
                return m_timer.hasElapsed(traj.getTotalTimeSeconds());
            }

            public void end(boolean interrupted) {
                m_timer.stop();
            }

            private Trajectory getTrajectory() {
                if (useDynamic) {
                    return dynamicTrajectory.apply(startPose);
                } else {
                    return trajectory.get();
                }
            }
        };
    }

    private Command followTrajectory(Function<Pose2d, Trajectory> dynamicTrajectory,
            Supplier<Rotation2d> desiredRotation,
            DoubleSupplier desiredRotationSpeed, boolean useRotSpeed) {
        return followTrajectory(dynamicTrajectory, MemoizedSupplier.memoize(() -> null), true, desiredRotation,
                desiredRotationSpeed, useRotSpeed);
    }

    private Command followTrajectory(MemoizedSupplier<Trajectory> trajectory, Supplier<Rotation2d> desiredRotation,
            DoubleSupplier desiredRotationSpeed, boolean useRotSpeed) {
        return followTrajectory((p) -> null, trajectory, false, desiredRotation, desiredRotationSpeed, useRotSpeed);
    }

    private Command followTrajectory(MemoizedSupplier<Trajectory> trajectory, Supplier<Rotation2d> desiredRotation) {
        return followTrajectory(trajectory, desiredRotation, () -> 0, false);
    }

    public Command followTrajectory(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        return followTrajectory(MemoizedSupplier.memoize(() -> trajectory), desiredRotation);
    }

    public Command followTrajectory(MemoizedSupplier<Trajectory> trajectory, DoubleSupplier desiredRotationSpeed) {
        return followTrajectory(trajectory, () -> new Rotation2d(), desiredRotationSpeed, true);
    }

    public Command followTrajectory(Trajectory trajectory, DoubleSupplier desiredRotationSpeed) {
        return followTrajectory(MemoizedSupplier.memoize(() -> trajectory), () -> new Rotation2d(),
                desiredRotationSpeed, true);
    }

    private Command followTrajectory(Function<Pose2d, Trajectory> trajectory, DoubleSupplier desiredRotationSpeed) {
        return followTrajectory(trajectory, () -> new Rotation2d(), desiredRotationSpeed, true);
    }

    public Command goToPointWithRot(ArrayList<Pose2d> waypoints, Supplier<Rotation2d> rotation, boolean noAvoidance) {
        Supplier<Trajectory> trajectory = () -> {
            try {
                if (noAvoidance) {
                    return generateTrajectoryNoAvoidance(getPose(), waypoints.get(waypoints.size() - 1));
                } else {
                    return generateTrajectory(waypoints);
                }
            } catch (ImpossiblePathException e) {
                DriverStation.reportWarning("Failed to generate path from " + getPose().getTranslation() + " to "
                        + waypoints.get(waypoints.size() - 1).getTranslation() + " with " + (waypoints.size() - 1)
                        + " waypoints.", e.getStackTrace());
                return null;
            }
        };
        return followTrajectory(MemoizedSupplier.memoize(trajectory), rotation).withName("Go To Point W/ Rot");
    }

    public Command goToPointWithRot(ArrayList<Pose2d> waypoints, Supplier<Rotation2d> rotation) {
        return goToPointWithRot(waypoints, rotation, false);
    }

    public Command goToPointWithRot(ArrayList<Pose2d> waypoints, Rotation2d rotation) {
        return goToPointWithRot(waypoints, () -> rotation);
    }

    public Command goToPointWithRot(Pose2d target, Supplier<Rotation2d> rotation) {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(target);
        return goToPointWithRot(waypoints, rotation);
    }

    public Command goToPointWithRot(Pose2d target, Rotation2d rotation) {
        return goToPointWithRot(target, () -> rotation);
    }

    public Command goToPointWithRotSpeed(ArrayList<Pose2d> waypoints, DoubleSupplier rotationSpeed,
            boolean noAvoidance) {
        Supplier<Trajectory> trajectory = () -> {
            try {
                if (noAvoidance) {
                    return generateTrajectoryNoAvoidance(getPose(), waypoints.get(waypoints.size() - 1));
                } else {
                    return generateTrajectory(waypoints);
                }
            } catch (ImpossiblePathException e) {
                DriverStation.reportWarning("Failed to generate path from " + getPose().getTranslation() + " to "
                        + waypoints.get(waypoints.size() - 1).getTranslation() + " with " + (waypoints.size() - 1)
                        + " waypoints.", e.getStackTrace());
                return null;
            }
        };
        return followTrajectory(MemoizedSupplier.memoize(trajectory), rotationSpeed)
                .withName("Go To Point W/ Rot Speed");
    }

    public Command goToPointWithRotSpeed(Supplier<Pose2d> target, DoubleSupplier rotationSpeed) {
        Supplier<Trajectory> trajectory = () -> {
            try {
                Trajectory test = generateTrajectory(target.get());
                return test;
            } catch (ImpossiblePathException e) {
                DriverStation.reportWarning("Failed to generate path from " + getPose().getTranslation() + " to "
                        + target.get().getTranslation() + ".", e.getStackTrace());
                e.printStackTrace();
                return null;
            }
        };
        return followTrajectory(MemoizedSupplier.memoize(trajectory), rotationSpeed)
                .withName("Go To Point W/ Rot Speed");
    }

    public Command goToPointWithRotSpeed(ArrayList<Pose2d> waypoints, DoubleSupplier rotationSpeed) {
        return goToPointWithRotSpeed(waypoints, rotationSpeed, false);
    }

    public Command goToAndTrackPoint(ArrayList<Pose2d> waypoints, Supplier<Pose2d> track, boolean flipped,
            boolean noAvoidance) {
        TargetRotationController controller = new TargetRotationController(track, flipped);

        DoubleSupplier inputRot = () -> controller.calculate(getPose(), getChassisSpeeds());

        return goToPointWithRotSpeed(waypoints, inputRot).withName("Go To and Track Point");
    }

    public Command goToAndTrackPoint(ArrayList<Pose2d> waypoints, Pose2d track, boolean flipped, boolean noAvoidance) {
        return goToAndTrackPoint(waypoints, () -> track, flipped, noAvoidance);
    }

    public Command goToAndTrackPoint(Pose2d target, Pose2d track, boolean flipped, boolean noAvoidance) {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(target);
        return goToAndTrackPoint(waypoints, () -> track, flipped, noAvoidance);
    }

    public Command goToAndTrackPoint(ArrayList<Pose2d> waypoints, Supplier<Pose2d> track, boolean flipped) {
        return goToAndTrackPoint(waypoints, track, flipped, false);
    }

    public Command goToAndTrackPoint(Supplier<Pose2d> waypoints, Supplier<Pose2d> track, boolean flipped) {
        TargetRotationController controller = new TargetRotationController(track, flipped);

        DoubleSupplier inputRot = () -> controller.calculate(getPose(), getChassisSpeeds());

        return goToPointWithRotSpeed(waypoints, inputRot).withName("Go To and Track Point");
    }

    public Command goToPointDriverRot(DoubleSupplier joystick) {
        DoubleSupplier targetRotSpeedSupplier = () -> rotSpeedFromJoystick(
                joystickToSpeed(joystick, true).getAsDouble(), true);
        return goToPointWithRotSpeed(new ArrayList<Pose2d>(), targetRotSpeedSupplier)
                .withName("Go To Point Driver Rot");
    }

    public Command goToNote(NoteDetection noteDetection, Pose2d targetRegion, double regionRadius,
            boolean cancelIfNone) {
        Supplier<Pose2d> notePose = noteDetection::getClosestNoteToRegion;
        TargetRotationController controller = new TargetRotationController(notePose, false);
        Function<Pose2d, Trajectory> trajSupplier = (p) -> this.generateTrajectoryNoAvoidance(p, notePose.get());

        Command followCommand = Commands.runOnce(() -> noteDetection.setRegion(targetRegion, regionRadius))
                .andThen(followTrajectory(trajSupplier,
                        () -> controller.calculate(getPose(), getChassisSpeeds())));
        if (cancelIfNone) {
            return followCommand.onlyIf(() -> noteDetection.hasNoteInRegion);
        }
        return followCommand;
    }

    public Command goToNote(NoteDetection noteDetection, boolean cancelIfNone) {
        Supplier<Pose2d> notePose = noteDetection::getClosestNote;
        TargetRotationController controller = new TargetRotationController(notePose, false);
        Function<Pose2d, Trajectory> trajSupplier = (p) -> this.generateTrajectoryNoAvoidance(p, notePose.get());

        Command followCommand = followTrajectory(trajSupplier,
                () -> controller.calculate(getPose(), getChassisSpeeds()));
        if (cancelIfNone) {
            return followCommand.onlyIf(() -> noteDetection.hasNote);
        }
        return followCommand;
    }
}
