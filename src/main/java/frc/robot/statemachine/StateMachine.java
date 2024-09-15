package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.DriverAssistState;
import frc.robot.statemachine.states.tele.ManualState;

public class StateMachine extends StateMachineBase {
    public StateMachine(CommandXboxController driverController, CommandXboxController operatorController) {
        super();

        State teleop = new TeleState(this);
        State auto = new AutoState(this);
        State test = new TestState(this);
        State disabled = new DisabledState(this);

        // Set the initial state
        currentState = disabled;

        // Teleop
        ManualState manual = new ManualState(this);
        DriverAssistState driverAssist = new DriverAssistState(this);

        teleop.withModeTransitions(disabled, teleop, auto, test)
                .withDefaultChild(manual)
                .withChild(driverAssist, driverController.rightTrigger(), 0);

        manual.withTransition(driverAssist, driverController.rightTrigger(), 0);
        driverAssist.withTransition(manual, driverController.rightTrigger().negate(), 0);

        // Auto
        auto.withModeTransitions(disabled, teleop, auto, test);

        // Test
        test.withModeTransitions(disabled, teleop, auto, test);

        // Disabled
        disabled.withModeTransitions(disabled, teleop, auto, test);
    }
}
