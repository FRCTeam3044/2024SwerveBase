// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AutoAiming;

/**
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final NoteDetection m_noteDetection = new NoteDetection();
    public static AutoAiming m_autoAiming;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    public static final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        try {
            m_autoAiming = new AutoAiming();
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Unable to load auto aiming data, check that it is in the right path", false);
            throw new RuntimeException(e);
        }

        m_robotDrive.setDefaultCommand(new ManualDriveCommand(this, m_robotDrive, m_driverController));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_driverController.leftTrigger().whileTrue(new GoToNoteCommand(m_robotDrive, m_noteDetection));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

}