package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class AutoSegments {
    public static Command shootNote() {
        AutoTriggers triggers = new AutoTriggers();

        triggers.hasNote().whileTrue(AutoCommands.driveToShootingZone());

        triggers.hasNote().and(triggers.inShootingZone())
                .onTrue(Commands.sequence(AutoCommands.aimAndShoot(), triggers.end()));

        return triggers;
    }

    public static Command scoreNote(Translation2d notePos) {
        Pose2d notePose = new Pose2d(notePos, new Rotation2d());
        AutoTriggers triggers = new AutoTriggers();

        Trigger canPickupNote = triggers.nearLocation(notePos).and(triggers.noteDetectedNear(notePos));
        triggers.autoEnabled().and(canPickupNote.negate())
                .whileTrue(RobotContainer.m_robotDrive.goToAndTrackPoint(notePose, notePose, false, false));

        canPickupNote.whileTrue(AutoCommands.pickupNoteAt(notePos));

        triggers.nearLocation(notePos).and(triggers.noteDetectedNear(notePos).negate())
                .whileTrue(triggers.endAfter(0.2));

        return triggers.raceWith(shootNote());
    }
}
