package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TargetRotationController {
    private final double kFF;
    private final double timestep;
    private final ProfiledPIDController pidController;

    private final double targetX;
    private final double targetY;

    public TargetRotationController(double kFF, double timestep, ProfiledPIDController pidController, double targetX,
            double targetY) {
        this.kFF = kFF;
        this.timestep = timestep;
        this.pidController = pidController;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    /*
     * Calculate current rotation error
     * e1 = atan2(pos.x - speakerX, pos.y - speakerY) - rot
     * Calculate predicted position in t seconds
     * predicted = (pos.x + (vel.x * t), pos.y + (vel.y * t))
     * Calculate predicted rotational error in t seconds
     * e2 = atan2(predicted.x - speakerX, predicted.y - speakerY) - rot
     * Calculate change in error
     * Δe = e2 - e1
     * Rotation Output: fb(e1) - kF * (Δe/t)
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
        double targetAngle = Math.atan2(position.getX() - targetX, position.getY() - targetY);
        double e1 = calculateError(targetAngle, position.getRotation().getRadians());

        double predictedX = position.getX() + (currentSpeeds.vxMetersPerSecond * timestep);
        double predictedY = position.getY() + (currentSpeeds.vyMetersPerSecond * timestep);
        double predictedRot = Math.atan2(predictedX - targetX, predictedY - targetY);

        double e2 = calculateError(predictedRot, position.getRotation().getRadians());
        double deltaE = e2 - e1;
        return pidController.calculate(e1) - (kFF * (deltaE / timestep));
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
