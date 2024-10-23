package frc.robot.statemachine;

import java.util.function.BooleanSupplier;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.smart.GetToShootingZoneState;
import frc.robot.statemachine.states.smart.PickupNoteAuto;
import frc.robot.statemachine.states.smart.ShootState;

public class AutoBuilder {
        // This is kinda gross but the goal is to jsonify this (hence the jsonobjects
        // for parameters)
        public static void congigureAutoOne(StateMachineBase machine, State auto) {
                auto.withNoChildren();

                JSONObject pickNote1Params = new JSONObject();
                pickNote1Params.put("notePos", JSONUtils.fromCoords(8, 2));
                State pickupNote1 = (new PickupNoteAuto(machine, pickNote1Params)).withName("PickupNote1");
                GetToShootingZoneState getToZone1 = new GetToShootingZoneState(machine);
                ShootState shoot1 = new ShootState(machine);
                auto.withDefaultChild(pickupNote1);
                pickupNote1.withTransition(getToZone1, RobotContainer.intake::hasNote, 0);
                getToZone1.withTransition(shoot1,
                                () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);

                Translation2d note2 = new Translation2d(2.87, 5.55);
                JSONObject pickNote2Params = new JSONObject();
                pickNote2Params.put("notePos", JSONUtils.fromTranslation2d(note2));
                State pickupNote2 = (new PickupNoteAuto(machine, pickNote2Params)).withName("PickupNote2");
                BooleanSupplier nearLocation = Triggers.nearLocation(note2, 0.3);
                pickupNote1.withTransition(pickupNote2,
                                () -> {
                                        System.out.println("Checking near location. Has note: "
                                                        + RobotContainer.m_noteDetection.hasNote + " Near location: "
                                                        + nearLocation.getAsBoolean());
                                        return !RobotContainer.m_noteDetection.hasNote && nearLocation.getAsBoolean();
                                });
                GetToShootingZoneState getToZone2 = new GetToShootingZoneState(machine);
                ShootState shoot2 = new ShootState(machine);
                pickupNote2.withTransition(getToZone2, RobotContainer.intake::hasNote, 0);
                getToZone2.withTransition(shoot2,
                                () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);

                shoot1.withTransition(pickupNote2, () -> !RobotContainer.intake.hasNote(), 0);
                // Loop :D
                shoot2.withTransition(pickupNote1, () -> !RobotContainer.intake.hasNote(), 0);

        }
}
