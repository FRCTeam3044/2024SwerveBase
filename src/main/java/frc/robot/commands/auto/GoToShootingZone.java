package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.TrackPointCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutoTargetUtils;
import me.nabdev.pathfinding.structures.ObstacleGroup;
import me.nabdev.pathfinding.structures.Vertex;

public class GoToShootingZone extends Command {

    private boolean failed = false;
    private final DriveSubsystem m_driveSubsystem;
    private Pose2d m_target;
    private Command next = null;

    public GoToShootingZone(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_target = null;
    }

    public GoToShootingZone(DriveSubsystem driveSubsystem, Pose2d target) {
        m_driveSubsystem = driveSubsystem;
        m_target = target;
    }

    @Override
    public void initialize() {
        System.out.println("Go To Shooting Zone Initialized");
        failed = false;
        ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
        if (shootingZone == null) {
            System.out.println("Unable to retrieve Shooting Zone");
            failed = true;
            return;
        }
        Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        if (m_target == null && shootingZone.isInside(robotPos)) {
            System.out.println("Already inside Shooting zone, aiming");
            next = new TrackPointCommand(m_driveSubsystem, trackPoint, true);
        } else {
            if (m_target == null) {
                System.out.println("Finding nearest point on shooting zone");
                m_target = shootingZone.calculateNearestPoint(robotPos).asPose2d();
            }
            GoToAndTrackPointCommand travelToPoint = new GoToAndTrackPointCommand(m_target, trackPoint,
                    m_driveSubsystem, true, true);
            TrackPointCommand trackPointCmd = new TrackPointCommand(m_driveSubsystem, trackPoint, true);
            next = Commands.sequence(travelToPoint, trackPointCmd);
        }

        next.schedule();
    }

    @Override
    public boolean isFinished() {
        return failed || (next != null && next.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("GoToShootingZone ended " + interrupted);
    }
}
