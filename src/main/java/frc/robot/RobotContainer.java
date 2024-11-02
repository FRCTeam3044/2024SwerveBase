// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AutoAiming;

import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;

/**
 * This class is where the bulk of the robot should be declared. Since
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

        public static final ClimberSubsystem climber = new ClimberSubsystem();
        public static final IntakeSubsystem intake = new IntakeSubsystem();
        public static final TransitSubsystem transit = new TransitSubsystem();
        public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public static final ShooterSubsystem shooter = new ShooterSubsystem();

        public StateMachine stateMachine = new StateMachine(m_driverController, m_operatorController);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureTestBindings();
                try {
                        m_autoAiming = new AutoAiming(true);
                } catch (FileNotFoundException e) {
                        DriverStation.reportError("Unable to load auto aiming data, check that it is in the right path",
                                        false);
                        throw new RuntimeException(e);
                }

                climber.setDefaultCommand(climber
                                .moveClimberJoysticks(() -> -m_operatorController.getLeftY(),
                                                m_operatorController::getRightY)
                                .onlyWhile(teleop()).withName("Move Climber"));
        }

        private void configureTestBindings() {

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand(String autoName) {
                return null;
        }

}