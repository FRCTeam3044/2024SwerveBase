package frc.robot.tropath.robotprofile;

public class RobotProfile {
    private double maxVelocity,
            maxAcceleration,
            maxRotationalVelocity,
            maxRotationalAcceleration,
            length,
            width;
    private double safteyMultiplier = .8;

    /**
     * Constructs a robot profile for the robot's dimensions and given maximums. For
     * calculated
     * maximums based on your robot's characteristics, use
     * {@link #RobotProfile(double, double,
     * double, double, Motor)}.
     *
     * @param maxVelocity               The maximum velocity in meters per second to
     *                                  set.
     * @param maxAcceleration           The maximum acceleration in meters per
     *                                  second squared to set.
     * @param maxRotationalVelocity     The maximum rotational velocity in radians
     *                                  per second to set.
     * @param maxRotationalAcceleration The maximum rotational acceleration in
     *                                  radians per second
     *                                  squared to set.
     * @param length                    The length of the robot in meters, including
     *                                  bumpers.
     * @param width                     The width of the robot in meters, including
     *                                  bumpers.
     */
    public RobotProfile(
            double maxVelocity,
            double maxAcceleration,
            double maxRotationalVelocity,
            double maxRotationalAcceleration,
            double length,
            double width) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxRotationalVelocity = maxRotationalVelocity;
        this.maxRotationalAcceleration = maxRotationalAcceleration;
        this.length = length;
        this.width = width;
        this.safteyMultiplier = 1;
    }

    /**
     * Constructs a robot profile with given dimensions as well as calculating
     * values from given
     * physical characteristics. If these calculated maximums seem faster than is
     * safe, use {@link
     * #setSafteyMultiplier}.
     *
     * @param robotMass     The mass of the robot in kilograms.
     * @param wheelDiameter The diameter of a wheel on the robot in meters.
     * @param length        The length of the robot in meters, including bumpers.
     * @param width         The width of the robot in meters, including bumpers.
     * @param driveMotor    The {@link Motor} used for driving.
     */
    public RobotProfile(
            double robotMass, double wheelDiameter, double length, double width, Motor driveMotor) {
        this.length = length;
        this.width = width;

        maxVelocity = driveMotor.getFreeSpeed() * Math.PI * wheelDiameter / 60;
        maxAcceleration = driveMotor.getStallTorque()
                / (wheelDiameter / 2)
                / robotMass
                * 4; // We have four swerve modules

        double robotRadius = 2 * Math.max(length, width) / Math.sqrt(2);
        maxRotationalVelocity = maxVelocity / robotRadius;
        maxRotationalAcceleration = maxAcceleration / robotRadius;
    }

    /**
     * @return The maximum velocity in meters per second.
     */
    public double getMaxVelocity() {
        return maxVelocity * safteyMultiplier;
    }

    /**
     * @param maxVelocity The maximum velocity in meters per second to set.
     */
    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    /**
     * @return The maximum acceleration in meters per second squared.
     */
    public double getMaxAcceleration() {
        return maxAcceleration * safteyMultiplier;
    }

    /**
     * @param maxAcceleration The maximum acceleration in meters per second squared
     *                        to set.
     */
    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * @return The rotational velocity in radians per second.
     */
    public double getMaxRotationalVelocity() {
        return maxRotationalVelocity * safteyMultiplier;
    }

    /**
     * @param maxRotationalVelocity The rotational velocity in radians per second to
     *                              set.
     */
    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        this.maxRotationalVelocity = maxRotationalVelocity;
    }

    /**
     * @return The rotational acceleration in radians per second squared.
     */
    public double getMaxRotationalAcceleration() {
        return maxRotationalAcceleration * safteyMultiplier;
    }

    /**
     * @param maxRotationalAcceleration The rotational acceleration in radians per
     *                                  second squared to
     *                                  set.
     */
    public void setMaxRotationalAcceleration(double maxRotationalAcceleration) {
        this.maxRotationalAcceleration = maxRotationalAcceleration;
    }

    /**
     * @return The length of the robot in meters.
     */
    public double getLength() {
        return length;
    }

    /**
     * @param length The length of the robot in meters to set.
     */
    public void setLength(double length) {
        this.length = length;
    }

    /**
     * @return The width of the robot.
     */
    public double getWidth() {
        return width;
    }

    /**
     * @param width The width of the robot in meters to set.
     */
    public void setWidth(double width) {
        this.width = width;
    }

    /**
     * @return The safety multiplier.
     */
    public double getSafteyMultiplier() {
        return safteyMultiplier;
    }

    /**
     * @param safteyMultiplier The safety multiplier to set.
     */
    public RobotProfile setSafteyMultiplier(double safteyMultiplier) {
        this.safteyMultiplier = safteyMultiplier;
        return this;
    }

    @Override
    public String toString() {
        return "RobotProfile [maxVelocity="
                + getMaxVelocity()
                + ", maxAcceleration="
                + getMaxAcceleration()
                + ", maxRotationalVelocity="
                + getMaxRotationalVelocity()
                + ", maxRotationalAcceleration="
                + getMaxRotationalAcceleration()
                + ", length="
                + getLength()
                + ", width="
                + getWidth()
                + ", safteyMultiplier="
                + getSafteyMultiplier()
                + "]";
    }
}
