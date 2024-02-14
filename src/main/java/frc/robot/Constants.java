// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final class DriveConstants {
                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final ConfigurableParameter<Double> kMaxSpeedMetersPerSecond = new ConfigurableParameter<Double>(
                                4.8,
                                "Max Speed");
                public static final ConfigurableParameter<Double> kMaxAngularSpeed = new ConfigurableParameter<Double>(
                                2 * Math.PI,
                                "Max Angular Speed");
                public static final ConfigurableParameter<Double> kDirectionSlewRate = new ConfigurableParameter<Double>(
                                1.2,
                                "Direction Slew rate"); // radians per second
                public static final double kMagnitudeSlewRate = 1.8; // radians per second
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(37);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(37);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

                // SPARK MAX CAN IDs
                public static final int kFrontLeftDrivingCanId = 17;
                public static final int kRearLeftDrivingCanId = 12;
                public static final int kFrontRightDrivingCanId = 15;
                public static final int kRearRightDrivingCanId = 13;

                public static final int kFrontLeftTurningCanId = 18;
                public static final int kRearLeftTurningCanId = 11;
                public static final int kFrontRightTurningCanId = 16;
                public static final int kRearRightTurningCanId = 14;

                public static final boolean kGyroReversed = false;

                public static final ConfigurableParameter<Boolean> kFieldRelative = new ConfigurableParameter<Boolean>(
                                true,
                                "Field Relative");
                public static final ConfigurableParameter<Boolean> kRateLimit = new ConfigurableParameter<Boolean>(true,
                                "Rate Limit");

                public static final ConfigurableParameter<Double> kStatemachineDebounce = new ConfigurableParameter<Double>(
                                0.1, "Statemachine Debounce");
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 14;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction) / 60.0; // meters per second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 40; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }

        public static final class CANConstants {
                public static final int kShooterTopMotorPort = 1;
                public static final int kShooterBottomMotorPort = 2;

                public static final int kTransitMotorPort = 3;
                public static final int kTransitSensorPort = 4;

                public static final int kIntakeSensorPort = 5;
                public static final int kIntakeTopMotorPort = 6;
                public static final int kIntakeBottomMotorPort = 7;

                public static final int kClimberLeftClimberMotorPort = 8;
                public static final int kClimberRightClimberMotorPort = 9;

                public static final int kElevatorMotorOnePort = 21;
                public static final int kElevatorMotorTwoPort = 20;
                public static final int kElevatorTopLimitSwitch = 23;
                public static final int kElevatorBottomLimitSwitch = 22;
        }

        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;
                public static final int kOperatorControllerPort = 1;
                public static final ConfigurableParameter<Double> kDriveDeadband = new ConfigurableParameter<Double>(
                                0.05,
                                "Driver Deadband");
        }

        public static final class PathfindingConstants {
                // I hate this. Java sucks
                public static void initialize() {
                        // This method does nothing but touch the ConfigurableParameter objects,
                        // forcing the class to be loaded and the objects to be initialized.
                        System.out.println(kMaxSpeedMetersPerSecond);
                }

                public static final ConfigurableParameter<Double> kMaxSpeedMetersPerSecond = new ConfigurableParameter<Double>(
                                3.0,
                                "Pathfinding Max Speed");
                public static final ConfigurableParameter<Double> kMaxAccelerationMetersPerSecondSquared = new ConfigurableParameter<Double>(
                                3.0, "Pathfinding Max Acceleration");
                // BOTH MAX ANGULAR SPEED & ACCEL REQUIRE CODE REBOOT TO CHANGE!!!
                public static final ConfigurableParameter<Double> kMaxAngularSpeedRadiansPerSecond = new ConfigurableParameter<Double>(
                                Math.PI, "Pathfinding Max Angular Speed");
                public static final ConfigurableParameter<Double> kMaxAngularAccelerationRadiansPerSecondSquared = new ConfigurableParameter<Double>(
                                Math.PI, "Pathfinding Max Angular Acceleration");

                public static final PIDController kPathfindingXController = new ConfigurablePIDController(1, 0, 0,
                                "Pathfinding X Controller");
                public static final PIDController kPathfindingYController = new ConfigurablePIDController(1, 0, 0,
                                "Pathfinding Y Controller");
                public static final ProfiledPIDController kPathfindingThetaController = new ConfigurableProfiledPIDController(
                                1,
                                0,
                                0,
                                // new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond.get(),
                                // kMaxAngularAccelerationRadiansPerSecondSquared.get()),
                                new TrapezoidProfile.Constraints(0, 0),
                                "Pathfinding Theta Controller");

                public static final ConfigurableParameter<Double> kRotationTimestep = new ConfigurableParameter<Double>(
                                0.02,
                                "Rotation FF Timestep");
                public static final ConfigurableParameter<Double> kRotationFF = new ConfigurableParameter<Double>(0.1,
                                "Rotation FF");

        }

        // TODO: Get real values for these
        public static final class PathfindingTargets {
                public static final Pose2d RED_SOURCE = new Pose2d(1, 1, new Rotation2d(0));
                public static final Pose2d BLUE_SOURCE = new Pose2d(1, 15.5, new Rotation2d(0));

        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
        }

        public static final class ShooterConstants {
                public static final ConfigurableParameter<Double> kShooterToleranceRPM = new ConfigurableParameter<Double>(
                                100.0, "Shooter Tolerance RPM");
        }

        public static final class ElevatorConstants {
                public static final ConfigurableParameter<Double> kElevatorTolerance = new ConfigurableParameter<Double>(
                                10.0, "Elevator Tolerance");
        }

        public static final class VisionConstants {
                // TODO: Update these values
                public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(4, 4, 8); // THESE ARE NOT CORRECT
                public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // THESE ARE NOT
                                                                                                   // CORRECT
                // array of active cameras
                public final static String[] activeCameras = {
                                "front",
                                // "back",
                                // "left",
                                // "right"
                };
                // array of camera transforms
                public final static Transform3d[] cameraTransforms = {
                                new Transform3d( // front
                                                new Translation3d(
                                                                0,
                                                                0,
                                                                0),
                                                new Rotation3d(
                                                                0,
                                                                0,
                                                                0)),
                };
        }

        public static final class DetectorConstants {
                public static double[][] cameraPointsArray = {
                                { 621, 538 },
                                { 625, 438 },
                                { 93, 450 },
                                { 1163, 469 }
                };
                public static double[][] fieldPointsArray = {
                                { 0.91, 0 },
                                { 1.82, 0 },
                                { 1.82, 0.91 },
                                { 1.82, -0.92 }
                };
        }
}