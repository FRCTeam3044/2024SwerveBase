package frc.robot.utils;

public class Range {
    private double min, max;

    public Range(double min, double max) {
        if (min < max) {
            this.min = min;
            this.max = max;
        } else {
            this.min = max;
            this.max = min;
        }
    }

    public double getMin() {
        return min;
    }

    public void setMin(double min) {
        this.min = min;
    }

    public double getMax() {
        return max;
    }

    public void setMax(double max) {
        this.max = max;
    }

    public int nearestDirection(double val) {
        if (val < min)
            return -1;
        if (val > max)
            return 1;
        return 0;
    }

    public double nearestValue(double val) {
        int direction = nearestDirection(val);
        if (direction == -1)
            return min;
        if (direction == 1)
            return max;
        return val;
    }

    public boolean contains(double val) {
        return Math.abs(nearestDirection(val)) == 0;
    }
}