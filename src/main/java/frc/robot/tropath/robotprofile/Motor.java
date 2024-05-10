package frc.robot.tropath.robotprofile;

public class Motor {
    private double stallTorque, freeSpeed;

    /**
     * Constructs a custom motor. In most cases, you should be able to just use
     * {@link Motor#CIM()},
     * {@link Motor#NEO()}, {@link Motor#FALCON()}, {@link Motor#VORTEX()}, or
     * {@link
     * Motor#KRAKENX60()}, and then use the {@link #gear()} method to get it in the
     * correct gear. If
     * you want to make a custom motor, {@link https://www.reca.lc/} should be used
     * to find the
     * values.
     *
     * @param realStallTorque The real stall torque in newton meters. At 12 volts
     *                        and 40 amps as found
     *                        in recalc.
     * @param freeSpeed       The free speed in rpm.
     */
    public Motor(double realStallTorque, double freeSpeed) {
        this.stallTorque = realStallTorque;
        this.freeSpeed = freeSpeed;
    }

    /**
     * @return The real stall torque in newton meters.
     */
    public double getStallTorque() {
        return stallTorque;
    }

    /**
     * Set the stall torque to something different than what had previously been
     * configured.
     *
     * @param realStallTorque The real stall torque in newton meters to set. At 12
     *                        volts and 40 amps
     *                        as found in recalc at {@link https://www.reca.lc/}.
     */
    public void setStallTorque(double realStallTorque) {
        this.stallTorque = realStallTorque;
    }

    /**
     * @return The free speed in rpm.
     */
    public double getFreeSpeed() {
        return freeSpeed;
    }

    /**
     * Set the free speed to something different than what had previously been
     * configured.
     *
     * @param freeSpeed The free speed in rpm.
     */
    public void setFreeSpeed(double freeSpeed) {
        this.freeSpeed = freeSpeed;
    }

    /**
     * Gears the motor.
     *
     * @param gearRatio The gear ratio can be a custom one (given as a double), but
     *                  in most cases, a
     *                  value in {@link Gear} should be able to be used.
     * @return A motor with the same settings as the current one but with the given
     *         gear.
     */
    public Motor gear(double gearRatio) {
        freeSpeed /= gearRatio;
        stallTorque *= gearRatio;
        return this;
    }

    /** Contains most gear ratios that would be used. */
    public static final class Gear {
        public static final double SDS_L1 = 8.14,
                SDS_L2 = 6.75,
                SDS_L3 = 6.12,
                SDS_L4 = 5.14,
                REV_LOW = 5.5,
                REV_MED = 5.08,
                REV_HIGH = 4.71,
                REV_EHIGH_1 = 4.5,
                REV_EHIGH_2 = 4.29,
                REV_EHIGH_3 = 4.00,
                REV_EHIGH_4 = 3.75,
                REV_EHIGH_5 = 3.56,
                WCP_X1_LOW = 7.85,
                WCP_X1_MED = 7.13,
                WCP_X1_HIGH = 6.54,
                WCP_X2_LOW = 6.56,
                WCP_X2_MED = 5.96,
                WCP_X2_HIGH = 5.46,
                WCP_X3_LOW = 5.14,
                WCP_X3_MED = 4.75,
                WCP_X3_HIGH = 4.41,
                WCP_XS1_12 = 6.00,
                WCP_XS1_13 = 5.54,
                WCP_XS1_14 = 5.14,
                WCP_XS2_14 = 4.71,
                WCP_XS2_15 = 4.40,
                WCP_XS2_16 = 4.13;
    }

    /**
     * @return A new motor with the correct settings for a CIM. Make sure to use
     *         {@link #gear(double)}
     *         to set it to the gear you are using.
     */
    public static Motor CIM() {
        return new Motor(.686, 5333);
    }

    /**
     * @return A new motor with the correct settings for a NEO. Make sure to use
     *         {@link #gear(double)}
     *         to set it to the gear you are using.
     */
    public static Motor NEO() {
        return new Motor(.701, 5880);
    }

    /**
     * @return A new motor with the correct settings for a Falcon. Make sure to use
     *         {@link
     *         #gear(double)} to set it to the gear you are using.
     */
    public static Motor FALCON() {
        return new Motor(.70003, 6380);
    }

    /**
     * @return A new motor with the correct settings for a Vortex. Make sure to use
     *         {@link
     *         #gear(double)} to set it to the gear you are using.
     */
    public static Motor VORTEX() {
        return new Motor(.621, 6784);
    }

    /**
     * @return A new motor with the correct settings for a Kraken. Make sure to use
     *         {@link
     *         #gear(double)} to set it to the gear you are using.
     */
    public static Motor KRAKENX60() {
        return new Motor(.746, 6000);
    }
}
