package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.DriverAssistState;
import frc.robot.statemachine.states.tele.GetToShootingZoneState;
import frc.robot.statemachine.states.tele.GetToSourceState;
import frc.robot.statemachine.states.tele.ManualState;
import frc.robot.statemachine.states.tele.PickupNoteState;
import frc.robot.statemachine.states.tele.ShootState;

public class StateMachine extends StateMachineBase {
    public StateMachine(CommandXboxController driverController, CommandXboxController operatorController) {
        super();
        State disabled = new DisabledState(this);
        // Set the initial state
        currentState = disabled;

        State teleop = new TeleState(this);
        State auto = new AutoState(this);
        State test = new TestState(this, driverController);

        // Teleop
        ManualState manual = new ManualState(this, driverController, operatorController);
        DriverAssistState driverAssist = new DriverAssistState(this);
        GetToSourceState getToSource = new GetToSourceState(this);
        PickupNoteState pickupNote = new PickupNoteState(this);
        GetToShootingZoneState getToShootingZone = new GetToShootingZoneState(this);
        ShootState shoot = new ShootState(this);

        teleop.withModeTransitions(disabled, teleop, auto, test)
                .withDefaultChild(manual)
                .withChild(driverAssist, driverController.rightTrigger(), 0);

        manual.withTransition(driverAssist, driverController.rightTrigger(), 0);
        driverAssist.withTransition(manual, driverController.rightTrigger().negate(), 0)
                .withDefaultChild(getToSource)
                .withChild(pickupNote, () -> RobotContainer.m_noteDetection.hasNote, 2)
                .withChild(getToShootingZone, RobotContainer.intake::hasNote, 1)
                .withChild(shoot, () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);
        getToSource.withTransition(pickupNote, () -> RobotContainer.m_noteDetection.hasNote);
        pickupNote.withTransition(getToShootingZone, RobotContainer.intake::hasNote, 0);
        pickupNote.withTransition(getToSource, () -> !RobotContainer.m_noteDetection.hasNote);
        getToShootingZone.withTransition(shoot, Triggers.inShootingZone());
        shoot.withTransition(getToSource, () -> !RobotContainer.intake.hasNote());

        // Auto
        auto.withModeTransitions(disabled, teleop, auto, test);

        // Test
        test.withModeTransitions(disabled, teleop, auto, test);

        // Disabled
        disabled.withModeTransitions(disabled, teleop, auto, test);
    }
}
