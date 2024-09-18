package frc.robot.statemachine.states;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TestState extends State {
        private static ConfigurableParameter<Double> kElevatorPIDControlTarget = new ConfigurableParameter<Double>(
                        0.0, "Elevator Test PID Target");

        public TestState(StateMachineBase stateMachine, CommandXboxController controller) {
                super(stateMachine);

                SmartXboxController testController = new SmartXboxController(controller, loop);
                testController.b().whileTrue(RobotContainer.intake.run());
                testController.y().whileTrue(RobotContainer.elevator.toAngle(kElevatorPIDControlTarget::get));
                testController.a()
                                .whileTrue(RobotContainer.elevator.test(() -> -testController.controller.getRightY()));
                testController.povDown().whileTrue(RobotContainer.shooter.slow());
                testController.povUp().whileTrue(RobotContainer.shooter.shoot());

                // If one trigger is pressed, move the climber. If both are pressed, don't move
                BooleanSupplier onlyOneTrigger = () -> testController.controller.getLeftTriggerAxis() > 0
                                ^ testController.controller.getRightTriggerAxis() > 0;
                DoubleSupplier climberOutput = () -> onlyOneTrigger.getAsBoolean()
                                ? (testController.controller.getRightTriggerAxis()
                                                - testController.controller.getLeftTriggerAxis())
                                                * ClimberConstants.kClimberManualSpeed.get()
                                : 0;
                testController.leftBumper().whileTrue(RobotContainer.climber.moveLeftClimber(climberOutput));
                testController.rightBumper().whileTrue(RobotContainer.climber.moveRightClimber(climberOutput));

                testController.x().whileTrue(RobotContainer.transit.run());

                onEnterTrg().onTrue(RobotContainer.m_robotDrive.test(controller::getLeftX, controller::getLeftY,
                                controller::getRightX, controller.a().or(controller.y())::getAsBoolean));
        }
}
