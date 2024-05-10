package frc.robot.tropath.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.tropath.robotprofile.RobotProfile;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;

/** A command to go to the given position. */
public class PathingCommand extends Command {
    private RobotProfile robotProfile;
    private Supplier<Pose2d> robotPose;
    private Consumer<ChassisSpeeds> drive;
    private Pathfinder pathfinder;
    private double velocity, rotationalVelocity = 0;
    private TrapezoidProfile rotationProfile;
    private Supplier<Pose2d> goalPoseSupplier;
    private double translationTolerance = .05, rotationTolerance = Math.PI / 32;
    private Field2d nextPoseFieldDisplay = new Field2d();
    private Field2d finalPoseFieldDisplay = new Field2d();
    private static final double dT = .02;
    private PathProfiler pathProfiler;
    private boolean linearPhysics = false;
    private ArrayList<CommandDuringPath> commands = new ArrayList<CommandDuringPath>();
    private double distanceLeft = Double.MAX_VALUE;

    /**
     * Constructs a PathingCommand. This method is called by the
     * {@link PathingCommandGenerator}.
     * Please use this generator to make a PathingCommand.
     */
    public PathingCommand(
            Supplier<Pose2d> goalSupplier,
            Supplier<Pose2d> currentPoseSupplier,
            Consumer<ChassisSpeeds> drive,
            RobotProfile robotProfile,
            Pathfinder pathfinder,
            Subsystem subsystem,
            double translationTolerance,
            double rotationTolerance,
            boolean linearPhysics) {
        this.goalPoseSupplier = goalSupplier;
        this.robotPose = currentPoseSupplier;
        this.drive = drive;
        this.linearPhysics = linearPhysics;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
        setRobotProfile(robotProfile);
        this.pathfinder = pathfinder;
        this.addRequirements(subsystem);
        SmartDashboard.putData("Next Pose", nextPoseFieldDisplay);
        SmartDashboard.putData("Final Pose", finalPoseFieldDisplay);
    }

    private PathingCommand setRobotProfile(RobotProfile profile) {
        this.robotProfile = profile;
        rotationProfile = new TrapezoidProfile(
                new Constraints(
                        profile.getMaxRotationalVelocity(), profile.getMaxRotationalAcceleration()));
        pathProfiler = new PathProfiler(profile.getMaxVelocity(), profile.getMaxAcceleration());
        return this;
    }

    /**
     * Add a command that will run in parallel with this command starting at the
     * given dist and
     * continuing until the goal position of this pathing command is reached.
     *
     * @param command The command to run.
     * @param dist    The distance from the goal at which to start running the
     *                command.
     * @return This pathing command.
     */
    public PathingCommand addCommandAtDist(Command command, double dist) {
        commands.add(new CommandDuringPath(command, -1, dist));
        return this;
    }

    /**
     * Add a command that will run in parallel with this command starting when this
     * command starts and
     * continuing until the robot reaches the given dist from the goal.
     *
     * @param command The command to run.
     * @param dist    The distance from the goal at which to stop running the
     *                command.
     * @return This pathing command.
     */
    public PathingCommand addCommandUntilDist(Command command, double dist) {
        commands.add(new CommandDuringPath(command, dist, Double.MAX_VALUE));
        return this;
    }

    /**
     * Add a command that will run in parallel with this command starting when the
     * robot reaches the
     * minDist from the goal position and ending when the robot gets to the maxDist
     * from the goal
     * position.
     *
     * @param command The command to run.
     * @param minDist The distance from the goal at which to stop running the
     *                command.
     * @param maxDist The distance from the goal at which to start running the
     *                command.
     * @return This pathing command.
     */
    public PathingCommand addCommandBetweenDist(Command command, double minDist, double maxDist) {
        if (minDist > maxDist) {
            double savedStart = minDist;
            minDist = maxDist;
            maxDist = savedStart;
        }
        commands.add(new CommandDuringPath(command, minDist, maxDist));
        return this;
    }

    public PathingCommand addCommandDistBasedCondition(Command command, Predicate<Double> isActive) {
        commands.add(new CommandDuringPath(command, isActive));
        return this;
    }

    @Override
    public void initialize() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        for (CommandDuringPath command : commands) {
            System.out.println("scheduling");
            Command endWait = new WaitUntilCommand(() -> !isScheduled() || !command.getIsActive(distanceLeft));
            scheduler.removeComposedCommand(endWait);
            scheduler.removeComposedCommand(command.getCommand());
            Command runCommand = command.getCommand().raceWith(endWait);
            scheduler.removeComposedCommand(runCommand);
            Command addedCommand = new WaitUntilCommand(() -> !isScheduled() || command.getIsActive(distanceLeft))
                    .andThen(runCommand);
            scheduler.schedule(addedCommand);
        }
    }

    public void execute() {
        finalPoseFieldDisplay.setRobotPose(goalPoseSupplier.get());
        double deltaRotation;
        deltaRotation = robotPose.get().getRotation().minus(goalPoseSupplier.get().getRotation()).getRadians();
        rotationalVelocity = rotationProfile.calculate(
                dT,
                new TrapezoidProfile.State(deltaRotation, rotationalVelocity),
                new TrapezoidProfile.State(0, 0)).velocity;
        Path path = null;
        long start = System.currentTimeMillis();
        try {
            path = pathfinder.generatePath(robotPose.get(), goalPoseSupplier.get());
            SmartDashboard.putNumber("Path generation time", System.currentTimeMillis() - start);
        } catch (ImpossiblePathException e) {
            e.printStackTrace();
            return;
        }
        Pose2d nextTargetPose;
        if (path.size() <= 1) {
            nextTargetPose = goalPoseSupplier.get();
        } else {
            nextTargetPose = path.get(1).asPose2d();
        }
        nextPoseFieldDisplay.setRobotPose(
                new Pose2d(nextTargetPose.getTranslation(), goalPoseSupplier.get().getRotation()));
        double dX = nextTargetPose.getX() - robotPose.get().getX(),
                dY = nextTargetPose.getY() - robotPose.get().getY();
        start = System.currentTimeMillis();
        if (linearPhysics) {
            velocity = pathProfiler.nextVelocityLinear(velocity, path.asPose2dList());
        } else {
            velocity = pathProfiler.getNextRobotSpeed(velocity, path.asPose2dList());
        }
        distanceLeft = pathProfiler.getDistanceToGoal();
        SmartDashboard.putNumber("Distance To Goal", pathProfiler.getDistanceToGoal());
        SmartDashboard.putNumber("Physics Time", System.currentTimeMillis() - start);
        SmartDashboard.putNumber("Velocity", velocity);
        double theta = Math.atan2(dY, dX);
        double xSpeed = velocity * Math.cos(theta);
        double ySpeed = velocity * Math.sin(theta);
        if (Double.isNaN(xSpeed) || Double.isNaN(ySpeed)) {
            xSpeed = 0;
            ySpeed = 0;
        }
        drive.accept(new ChassisSpeeds(xSpeed, ySpeed, rotationalVelocity));
    }

    public boolean isFinished() {
        return (robotPose.get().getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
                + velocity * velocity / 2 / robotProfile.getMaxAcceleration() < translationTolerance
                && Math.abs(
                        robotPose
                                .get()
                                .getRotation()
                                .minus(goalPoseSupplier.get().getRotation())
                                .getRadians())
                        + rotationalVelocity
                                * rotationalVelocity
                                / 2
                                / robotProfile.getMaxRotationalAcceleration() < rotationTolerance);
    }

    private class CommandDuringPath {
        private Command command;
        private Predicate<Double> isActivate;

        public CommandDuringPath(Command command, double minDistance, double maxDistance) {
            this.command = command;
            isActivate = (Double dist) -> dist < maxDistance && dist > minDistance;
        }

        public CommandDuringPath(Command command, Predicate<Double> isActivateGivenDist) {
            this.command = command;
            isActivate = isActivateGivenDist;
        }

        public Command getCommand() {
            return command;
        }

        public boolean getIsActive(double dist) {
            return isActivate.test(dist);
        }
    }
}
