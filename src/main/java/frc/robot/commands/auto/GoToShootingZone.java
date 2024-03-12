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
    private Command next = null;

    public GoToShootingZone(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        failed = false;
        ObstacleGroup shootingZone = AutoTargetUtils.getShootingZone();
        if (shootingZone == null) {
            failed = true;
            return;
        }
        Vertex robotPos = new Vertex(m_driveSubsystem.getPose());
        Pose2d trackPoint = AutoTargetUtils.getShootingTarget();
        if (shootingZone.isInside(robotPos)) {
            next = new TrackPointCommand(m_driveSubsystem, trackPoint, true);
        } else {
            Pose2d closestPoint = shootingZone.calculateNearestPoint(robotPos).asPose2d();
            GoToAndTrackPointCommand travelToPoint = new GoToAndTrackPointCommand(closestPoint, trackPoint,
                    m_driveSubsystem, true);
            TrackPointCommand trackPointCmd = new TrackPointCommand(m_driveSubsystem, trackPoint, true);
            next = Commands.sequence(travelToPoint, trackPointCmd);
        }
        next.schedule();
    }

    @Override
    public boolean isFinished() {
        return failed || (next != null && next.isFinished());
    }
}
