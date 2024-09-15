// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.reusable.AutoFactory;
import frc.robot.commands.StateMachineCommand;
import frc.robot.statemachine.AutoCommands;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.sim.SimStateMachine;
import frc.robot.utils.AutoAiming;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.ConditionalXboxController;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.io.FileNotFoundException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateMachineResetCommand;
import frc.robot.subsystems.TransitSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
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

        public static final ConditionalXboxController m_driverTeleController = new ConditionalXboxController(
                        m_driverController, teleop());
        public static final ConditionalXboxController m_operatorTeleController = new ConditionalXboxController(
                        m_operatorController,
                        teleop());
        public static final ConditionalXboxController m_testController = new ConditionalXboxController(
                        m_driverController,
                        test());

        public static final ClimberSubsystem climber = new ClimberSubsystem();
        public static final IntakeSubsystem intake = new IntakeSubsystem();
        public static final TransitSubsystem transit = new TransitSubsystem();
        public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
        public static final ShooterSubsystem shooter = new ShooterSubsystem();

        public StateMachine stateMachine = new StateMachine(m_driverController, m_operatorController);

        private static ConfigurableParameter<Double> kElevatorPIDControlTarget = new ConfigurableParameter<Double>(
                        0.0, "Elevator Test PID Target");

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
                                .moveClimberJoysticks(m_operatorController::getLeftY, m_operatorController::getRightY)
                                .onlyWhile(teleop()).withName("Move Climber"));
        }

        private void configureTestBindings() {
                m_testController.b().whileTrue(intake.run());
                m_testController.y().whileTrue(elevator.toAngle(kElevatorPIDControlTarget::get));
                m_testController.a().whileTrue(elevator.test(() -> -m_testController.controller.getRightY()));
                m_testController.povDown().whileTrue(shooter.slow());
                m_testController.povUp().whileTrue(shooter.shoot());

                // If one trigger is pressed, move the climber. If both are pressed, don't move
                BooleanSupplier onlyOneTrigger = () -> m_testController.controller.getLeftTriggerAxis() > 0
                                ^ m_testController.controller.getRightTriggerAxis() > 0;
                DoubleSupplier climberOutput = () -> onlyOneTrigger.getAsBoolean()
                                ? (m_testController.controller.getRightTriggerAxis()
                                                - m_testController.controller.getLeftTriggerAxis())
                                                * ClimberConstants.kClimberManualSpeed.get()
                                : 0;
                m_testController.leftBumper().whileTrue(climber.moveLeftClimber(climberOutput));
                m_testController.rightBumper().whileTrue(climber.moveRightClimber(climberOutput));

                m_testController.x().whileTrue(transit.run());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand(String autoName) {
                return AutoFactory.testAuto();
        }

}