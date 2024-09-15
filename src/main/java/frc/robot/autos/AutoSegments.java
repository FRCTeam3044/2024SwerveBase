package frc.robot.autos;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.statemachine.reusable.BTrigger;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoTargetUtils;

public class AutoSegments {
        public static Command shootNote() {
                AutoTriggers triggers = new AutoTriggers("Shoot Note", true);
                AtomicBoolean hasShot = new AtomicBoolean(false);

                // triggers.hasNote().whileTrue(AutoCommands.driveToShootingZone()
                // .andThen(RobotContainer.m_robotDrive.trackPoint(AutoTargetUtils::getShootingTarget,
                // true))
                // .withName("Get To Shooting Zone and Aim"));

                // triggers.hasNote().and(triggers.nearLocation(() -> {
                // return AutoTargetUtils.getShootingTarget().getTranslation();
                // }, ShooterConstants.kShooterSpinupRange.get()))
                // .whileTrue(RobotContainer.shooter.speaker().withName("Spin Up Shooter"));

                // triggers.hasNote().and(triggers.readyToShoot())
                // .onTrue(RobotContainer.transit.run().alongWith(Commands.runOnce(() -> {
                // hasShot.set(true);
                // })).onlyWhile(triggers.hasNote()).withName("Auto Shoot"));

                // triggers.noNote().and(hasShot::get).whileTrue(triggers.end());

                return triggers;
        }

        public static Command scoreNote(Translation2d notePos) {

                AutoTriggers triggers = new AutoTriggers("Score Note", true);

                return triggers.raceWith(shootNote());
        }
}
