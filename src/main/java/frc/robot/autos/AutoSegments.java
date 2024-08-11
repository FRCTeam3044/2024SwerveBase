package frc.robot.autos;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoTargetUtils;

public class AutoSegments {
    public static Command shootNote() {
        AutoTriggers triggers = new AutoTriggers();
        AtomicBoolean hasShot = new AtomicBoolean(false);

        triggers.hasNote().whileTrue(AutoCommands.driveToShootingZone()
                .andThen(RobotContainer.m_robotDrive.trackPoint(AutoTargetUtils::getShootingTarget, true))
                .withName("Get To Shooting Zone and Aim"));

        triggers.hasNote().and(triggers.nearLocation(() -> {
            return AutoTargetUtils.getShootingTarget().getTranslation();
        }, ShooterConstants.kShooterSpinupRange.get()))
                .whileTrue(RobotContainer.shooter.speaker().withName("Spin Up Shooter"));

        triggers.hasNote().and(triggers.readyToShoot())
                .onTrue(RobotContainer.transit.run().alongWith(Commands.runOnce(() -> {
                    hasShot.set(true);
                })).withName("Auto Shoot"));

        triggers.noNote().and(hasShot::get).whileTrue(triggers.end());

        return triggers.withName("Shoot Note");
    }

    public static Command scoreNote(Translation2d notePos) {
        Pose2d notePose = new Pose2d(notePos, new Rotation2d());
        AutoTriggers triggers = new AutoTriggers();

        Trigger canPickupNote = triggers.nearLocation(notePos).and(triggers.noteDetectedNear(notePos));
        triggers.autoEnabled().and(canPickupNote.negate()).and(triggers.hasNote().negate())
                .whileTrue(RobotContainer.m_robotDrive.goToAndTrackPoint(notePose, notePose, false, false));

        canPickupNote
                .onTrue(AutoCommands.pickupNoteAt(notePos).onlyWhile(triggers.hasNote().negate())
                        .withName("Auto Pickup Note At"));

        triggers.nearLocation(notePos).and(triggers.noteDetectedNear(notePos).negate())
                .whileTrue(triggers.endAfter(0.2));

        return triggers.raceWith(shootNote()).withName("Score Note");
    }
}
