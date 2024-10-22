package frc.robot.statemachine.states.smart;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.autos.reusable.AutoFactory;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class ManualState extends State {
        public ManualState(StateMachineBase stateMachine, CommandXboxController driverControllerRaw,
                        CommandXboxController operatorControllerRaw) {
                super(stateMachine);

                SmartXboxController driverController = new SmartXboxController(driverControllerRaw, loop);
                SmartXboxController operatorController = new SmartXboxController(operatorControllerRaw, loop);

                Command autoAimAndAlignCommand = RobotContainer.elevator.autoAim(RobotContainer.m_robotDrive)
                                .alongWith(RobotContainer.m_robotDrive.driveAndTrackPoint(driverControllerRaw::getLeftX,
                                                driverControllerRaw::getLeftY, AutoTargetUtils.getShootingTarget(),
                                                true));

                // When the menu button is pressed*
                driverController.x().whileTrue(RobotContainer.m_robotDrive.setXMode());
                driverController.b().whileTrue(AutoFactory.testAuto());

                operatorController.x().whileTrue(RobotContainer.intake.run());
                operatorController.y().whileTrue(RobotContainer.transit.run().alongWith(RobotContainer.intake.run()));
                operatorController.leftTrigger().whileTrue(RobotContainer.shooter.speaker());
                operatorController.leftBumper().whileTrue(RobotContainer.elevator.amp());
                operatorController.rightBumper().whileTrue(RobotContainer.shooter.amp());
                operatorController.rightTrigger().whileTrue(RobotContainer.shooter.lob());
                operatorController.a().whileTrue(RobotContainer.elevator.intake());
                operatorController.povDown().whileTrue(RobotContainer.intake.run(true));

                Command manualDrive = RobotContainer.m_robotDrive.manualDrive(driverControllerRaw::getLeftX,
                                driverControllerRaw::getLeftY, driverControllerRaw::getRightX,
                                driverController.rightTrigger()::getAsBoolean);

                driverController.leftTrigger().and(operatorController.a().negate())
                                .runWhileTrue(autoAimAndAlignCommand)
                                .runWhileFalse(manualDrive);
        }
}
