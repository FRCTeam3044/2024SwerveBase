// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class FollowTrajectoryCommand extends Command {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final Supplier<Double> m_desiredRotationSpeed;
    private final DriveSubsystem m_driveSubsystem;
    private final boolean useSuppliedRotSpeed;

    /**
     * Constructs a new FollowTrajectory command that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * raw module states from the position controllers which need to be put into a
     * velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param controller      The HolonomicDriveController for the drivetrain.
     * @param desiredRotation The angle that the drivetrain should be facing.
     *                        This is sampled at each
     *                        time step.
     * @param driveSubsystem  The subsystem to use to drive the robot.
     * @param requirements    The subsystems to require.
     */
    public FollowTrajectoryCommand(
            Trajectory trajectory,
            HolonomicDriveController controller,
            Supplier<Rotation2d> desiredRotation,
            DriveSubsystem driveSubsystem,
            SubsystemBase... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");
        m_driveSubsystem = requireNonNullParam(driveSubsystem, "driveSubsystem", "SwerveControllerCommand");
        m_desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");
        m_desiredRotationSpeed = null;
        useSuppliedRotSpeed = false;
        addRequirements(requirements);
    }

    /**
     * Constructs a new FollowTrajectory command that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * raw module states from the position controllers which need to be put into a
     * velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory           The trajectory to follow.
     * @param desiredRotationSpeed The speed that the drivetrain should be rotating.
     *                             This is sampled at each time step.
     * @param controller           The HolonomicDriveController for the drivetrain.
     * @param driveSubsystem       The subsystem to use to drive the robot.
     * @param requirements         The subsystems to require.
     */
    public FollowTrajectoryCommand(
            Trajectory trajectory,
            Supplier<Double> desiredRotationSpeed,
            HolonomicDriveController controller,
            DriveSubsystem driveSubsystem,
            SubsystemBase... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");
        m_driveSubsystem = requireNonNullParam(driveSubsystem, "driveSubsystem", "SwerveControllerCommand");
        m_desiredRotationSpeed = requireNonNullParam(desiredRotationSpeed, "desiredRotationSpeed",
                "SwerveControllerCommand");
        m_desiredRotation = () -> new Rotation2d();
        useSuppliedRotSpeed = true;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);
        Rotation2d desiredRotation = m_desiredRotation.get();
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_driveSubsystem.getPose(), desiredState,
                desiredRotation);
        if (useSuppliedRotSpeed) {
            targetChassisSpeeds.omegaRadiansPerSecond = m_desiredRotationSpeed.get();
        } else {
            targetChassisSpeeds.omegaRadiansPerSecond *= -1;
        }
        var targetModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        m_driveSubsystem.setModuleStates(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}