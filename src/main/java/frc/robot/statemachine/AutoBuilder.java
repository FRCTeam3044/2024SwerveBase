package frc.robot.statemachine;

import java.util.function.BooleanSupplier;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.JSONUtils;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoIdleState;
import frc.robot.statemachine.states.smart.GetToShootingZoneState;
import frc.robot.statemachine.states.smart.PickupNoteAuto;
import frc.robot.statemachine.states.smart.ShootState;
import frc.robot.utils.AutoTargetUtils;

public class AutoBuilder {
        public static double fieldCenter = 8.275;

        // This is kinda gross but the goal is to jsonify this (hence the jsonobjects
        // for parameters)
        public static void congigureAutoOne(StateMachineBase machine, State auto) {
                auto.withNoChildren();

                ShootState shootPreload = new ShootState(machine);

                JSONObject pickNote1Params = new JSONObject();
                Translation2d note1 = new Translation2d(2.87, 7);
                pickNote1Params.put("notePos", JSONUtils.fromTranslation2d(note1));
                State pickupNote1 = (new PickupNoteAuto(machine, pickNote1Params)).withName("PickupNote1");
                GetToShootingZoneState getToZone1 = new GetToShootingZoneState(machine);
                ShootState shoot1 = new ShootState(machine);

                auto.withDefaultChild(shootPreload).withChild(pickupNote1).withChild(getToZone1).withChild(shoot1);
                shootPreload.withTransition(pickupNote1, () -> !RobotContainer.intake.hasNote(), 0);
                pickupNote1.withTransition(getToZone1, RobotContainer.intake::hasNote, 0);
                getToZone1.withTransition(shoot1,
                                () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);

                BooleanSupplier nearNoteOneLocation = Triggers.nearLocation(note1, 0.1);

                Translation2d note2 = new Translation2d(2.87, 5.55);
                JSONObject pickNote2Params = new JSONObject();
                pickNote2Params.put("notePos", JSONUtils.fromTranslation2d(note2));
                State pickupNote2 = (new PickupNoteAuto(machine, pickNote2Params)).withName("PickupNote2");
                pickupNote1.withTransition(pickupNote2,
                                () -> !RobotContainer.m_noteDetection.hasNote && nearNoteOneLocation.getAsBoolean());
                GetToShootingZoneState getToZone2 = new GetToShootingZoneState(machine);
                ShootState shoot2 = new ShootState(machine);
                auto.withChild(shoot2).withChild(pickupNote2).withChild(getToZone2);
                pickupNote2.withTransition(getToZone2, RobotContainer.intake::hasNote, 0);
                getToZone2.withTransition(shoot2,
                                () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);

                shoot1.withTransition(pickupNote2, () -> !RobotContainer.intake.hasNote(), 0);

                Translation2d note3 = new Translation2d(2.87, 4.11);
                JSONObject pickNote3Params = new JSONObject();
                pickNote3Params.put("notePos", JSONUtils.fromTranslation2d(note3));
                State pickupNote3 = (new PickupNoteAuto(machine, pickNote3Params)).withName("PickupNote3");
                pickupNote2.withTransition(pickupNote3,
                                () -> !RobotContainer.m_noteDetection.hasNote
                                                && Triggers.nearLocation(note2, 0.1).getAsBoolean());
                GetToShootingZoneState getToZone3 = new GetToShootingZoneState(machine);
                ShootState shoot3 = new ShootState(machine);
                auto.withChild(shoot3).withChild(pickupNote3).withChild(getToZone3);
                pickupNote3.withTransition(getToZone3, RobotContainer.intake::hasNote, 0);
                getToZone3.withTransition(shoot3,
                                () -> RobotContainer.intake.hasNote() && Triggers.inShootingZone().getAsBoolean(), 0);

                shoot2.withTransition(pickupNote3, () -> !RobotContainer.intake.hasNote(), 0);
                shoot3.withTransition(new AutoIdleState(machine), () -> !RobotContainer.intake.hasNote(), 0);

        }

        public static Translation2d allianceFlip(Translation2d bluePos) {
                boolean isRed = AutoTargetUtils.redAlliance();
                if (isRed) {
                        return new Translation2d(2 * fieldCenter - bluePos.getX(), bluePos.getY());
                } else {
                        return bluePos;
                }
        }

        public static Translation2d allianceFlip(double x, double y) {
                return allianceFlip(new Translation2d(x, y));
        }
}
