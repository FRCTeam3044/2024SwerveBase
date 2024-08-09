package frc.robot.utils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PathfindingConstants;
import me.nabdev.oxconfig.ConfigurableParameter;

public class TargetRotationController {
    private DoubleSupplier targetX;
    private DoubleSupplier targetY;
    private boolean flipped;

    private static ConfigurableParameter<Double> minSpeed = new ConfigurableParameter<Double>(0.01,
            "Rot Controller Min Speed Rad/sec");
    private static ConfigurableParameter<Double> maxSpeed = new ConfigurableParameter<Double>(Math.PI,
            "Rot Controller Max Speed Rad/sec");
    private static ConfigurableParameter<Double> minError = new ConfigurableParameter<Double>(0.01,
            "Rot Controller Min Error Radians");

    public TargetRotationController(DoubleSupplier targetX, DoubleSupplier targetY, boolean flipped) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.flipped = flipped;
    }

    public TargetRotationController(Supplier<Pose2d> target, boolean flipped) {
        this.targetX = () -> target.get().getX();
        this.targetY = () -> target.get().getY();
        this.flipped = flipped;
    }

    /*
     * Calculate current rotation error
     * e1 = atan2(pos.x - speakerX, pos.y - speakerY) - rot
     * Calculate predicted position in t seconds
     * predicted = (pos.x + (vel.x * t), pos.y + (vel.y * t))
     * Calculate predicted rotational error in t seconds
     * e2 = atan2(predicted.x - speakerX, predicted.y - speakerY) - rot
     * Calculate change in error
     * deltae = e2 - e1
     * Rotation Output: fb(e1) - kF * (deltae/t)
     */

    /**
     * Calculate the rotation output
     * 
     * @param currentAngle  The current angle of the robot, in radians
     * @param position      The current position of the robot
     * @param currentSpeeds The current speeds of the robot
     * @return The next rotation output
     */
    public double calculate(Pose2d position, ChassisSpeeds currentSpeeds) {
        double currentRot = position.getRotation().getRadians();

        double timestep = PathfindingConstants.kRotationTimestep.get();
        double targetAngle = Math.atan2(targetY.getAsDouble() - position.getY(),
                targetX.getAsDouble() - position.getX());
        if (flipped) {
            targetAngle = targetAngle + Math.PI;
            targetAngle = MathUtil.angleModulus(targetAngle);
        }
        // SmartDashboard.putNumber("Current Angle",
        // position.getRotation().getRadians());
        // SmartDashboard.putNumber("Target Angle", targetAngle);
        double e1 = calculateError(targetAngle, currentRot);
        // SmartDashboard.putNumber("Current Rot Error", e1);

        currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, position.getRotation());
        double predictedX = position.getX() + (currentSpeeds.vxMetersPerSecond * timestep);
        double predictedY = position.getY() + (currentSpeeds.vyMetersPerSecond * timestep);
        double predictedRot = Math.atan2(targetY.getAsDouble() - predictedY, targetX.getAsDouble() - predictedX);
        if (flipped) {
            predictedRot = predictedRot + Math.PI;
            predictedRot = MathUtil.angleModulus(predictedRot);
        }
        double e2 = calculateError(predictedRot, currentRot);
        double deltaE = e2 - e1;
        // SmartDashboard.putNumber("Delta e", deltaE);
        double pidOut = PathfindingConstants.kPathfindingThetaController.calculate(e1);
        // SmartDashboard.putNumber("PID Out", pidOut);
        // SmartDashboard.putNumberArray("Rotation Target", targetPose);

        double ffTerm = PathfindingConstants.kRotationFF.get() * (deltaE / timestep);
        // SmartDashboard.putNumber("Rotation FF Output", ffTerm);
        // SmartDashboard.putNumber("kF", PathfindingConstants.kRotationFF.get());

        double finalOut = -(pidOut - ffTerm);

        if (Math.abs(finalOut) < minSpeed.get() && Math.abs(e1) > minError.get()) {
            finalOut = Math.copySign(minSpeed.get(), finalOut);
        } else if (Math.abs(finalOut) > maxSpeed.get()) {
            finalOut = Math.copySign(maxSpeed.get(), finalOut);
        }

        SmartDashboard.putNumber("Rot Controller Output", finalOut);
        return finalOut;
    }

    /**
     * Calculate the optimized angle error in radians. This will output the shortest
     * error, like -20 in the case of target 350 and current 10
     * 
     * @param target  The target angle, in radians
     * @param current The current angle, in radians
     * @return The optimized angle error, in radians
     */
    private double calculateError(double target, double current) {
        double error = target - current;
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }
        return MathUtil.angleModulus(error);
    }
}
