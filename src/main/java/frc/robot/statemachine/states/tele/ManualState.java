package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.autos.reusable.AutoFactory;
import frc.robot.statemachine.Triggers;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.utils.AutoTargetUtils;

public class ManualState extends State {
    Triggers triggers = new Triggers(loop);

    public ManualState(StateMachineBase stateMachine, CommandXboxController driverController,
            CommandXboxController operatorController) {
        super(stateMachine);

        Command autoAimAndAlignCommand = RobotContainer.elevator.autoAim(RobotContainer.m_robotDrive)
                .alongWith(RobotContainer.m_robotDrive.driveAndTrackPoint(driverController::getLeftX,
                        driverController::getLeftY, AutoTargetUtils.getShootingTarget(), true));

        driverController.leftTrigger(0.5, loop)
                .whileTrue(autoAimAndAlignCommand.onlyIf(() -> (!operatorController.getHID().getAButton())));
        // When the menu button is pressed*
        driverController.x(loop).whileTrue(RobotContainer.m_robotDrive.setXMode());
        driverController.b(loop).whileTrue(AutoFactory.testAuto());

        operatorController.x(loop).whileTrue(RobotContainer.intake.run());
        operatorController.y(loop).whileTrue(RobotContainer.transit.run().alongWith(RobotContainer.intake.run()));
        operatorController.leftTrigger(0.5, loop).whileTrue(RobotContainer.shooter.speaker());
        operatorController.leftBumper(loop).whileTrue(RobotContainer.elevator.amp());
        operatorController.rightBumper(loop).whileTrue(RobotContainer.shooter.amp());
        operatorController.rightTrigger(0.5, loop).whileTrue(RobotContainer.shooter.lob());
        operatorController.a(loop).whileTrue(RobotContainer.elevator.intake());
        operatorController.pov(0, 180, loop).whileTrue(RobotContainer.intake.run(true));
        // operatorController.b()
        // .whileTrue(AutoCommands.pickupNoteCmd().getPickupNoteCommand().onlyIf(() ->
        // m_noteDetection.hasNote));
        active().whileTrue(
                RobotContainer.m_robotDrive.manualDrive(driverController::getLeftX, driverController::getLeftY,
                        driverController::getRightX, driverController.rightTrigger()::getAsBoolean));
    }
}
