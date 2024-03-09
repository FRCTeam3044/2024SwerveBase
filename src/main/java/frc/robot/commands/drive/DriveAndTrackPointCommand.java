package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.AutoTargetUtils;
import frc.robot.utils.TargetRotationController;

/**
 * A command that allows the driver to control the drivebase using the
 * XboxController.
 */
public class DriveAndTrackPointCommand extends Command {
    private final CommandXboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    private final boolean isSimulation;

    private final TargetRotationController m_targetRotController;

    private Supplier<Pose2d> targetSupplier;
    private Pose2d lastTarget;
    private final boolean flipped;

    /**
     * Creates a new DriveAndTrackPointCommand, defaulting to tracking the speaker.
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     */
    public DriveAndTrackPointCommand(DriveSubsystem driveSubsystem, CommandXboxController driverController,
            boolean flipped) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
        this.flipped = flipped;
        isSimulation = RobotBase.isSimulation();
        Pose2d target = AutoTargetUtils.getShootingTarget();
        lastTarget = target;
        targetSupplier = AutoTargetUtils::getShootingTarget;
        if (target == null) {
            m_targetRotController = new TargetRotationController(0, 0, flipped);
        } else {
            m_targetRotController = new TargetRotationController(target.getX(), target.getY(), flipped);
        }
        addRequirements(m_robotDrive);
    }

    /**
     * Creates a new DriveAndTrackPointCommand
     * 
     * @param driveSubsystem   The drive subsystem this command will run on
     * @param driverController The XboxController that provides the input for the
     *                         drive command
     * @param target           The target to track
     */
    public DriveAndTrackPointCommand(DriveSubsystem driveSubsystem, CommandXboxController driverController,
            Pose2d target, boolean flipped) {
        m_robotDrive = driveSubsystem;
        m_driverController = driverController;
        this.flipped = flipped;
        isSimulation = RobotBase.isSimulation();
        targetSupplier = () -> target;
        m_targetRotController = new TargetRotationController(target.getX(), target.getY(), flipped);
        addRequirements(m_robotDrive);
    }

    public DriveAndTrackPointCommand(Supplier<Pose2d> target, CommandXboxController driverController,
            DriveSubsystem driveSubsystem, boolean flipped) {
        this.flipped = flipped;
        targetSupplier = target;
        try {
            lastTarget = target.get();
        } catch (Throwable e) {
            e.printStackTrace();
        }
        isSimulation = RobotBase.isSimulation();
        m_driverController = driverController;
        m_robotDrive = driveSubsystem;
        m_targetRotController = new TargetRotationController(lastTarget.getX(), lastTarget.getY(), flipped);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        if (lastTarget != targetSupplier.get()) {
            Pose2d target = targetSupplier.get();
            m_targetRotController.setTargetX(target.getX());
            m_targetRotController.setTargetY(target.getY());
        }
    }

    @Override
    public void execute() {
        double inputX = MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get());
        double inputY = MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get());
        double inputRot = m_targetRotController.calculate(m_robotDrive.getPose(), m_robotDrive.getChassisSpeeds());

        inputX = inputX * inputX * Math.signum(inputX);
        inputY = inputY * inputY * Math.signum(inputY);
        inputRot = inputRot * inputRot * Math.signum(inputRot);

        if (isSimulation) {
            m_robotDrive.drive(inputX, -inputY, inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), true);
        } else {
            m_robotDrive.drive(inputY, inputX, inputRot, DriveConstants.kFieldRelative.get(),
                    DriveConstants.kRateLimit.get(), true);
        }
    }
}
