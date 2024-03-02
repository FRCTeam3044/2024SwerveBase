package frc.robot.utils;

import java.util.NavigableMap;

public class LinearInterpolation {
    private NavigableMap<Double, Double> m_map;

    public LinearInterpolation(NavigableMap<Double, Double> map) {
        m_map = map;
    }

    public LinearInterpolation(double[] x, double[] y) {
        for (int i = 0; i < x.length; i++) {
            m_map.put(x[i], y[i]);
        }
    }

    public double interpolate(double x) {
        if (m_map.containsKey(x)) {
            return m_map.get(x);
        }
        Double lower = m_map.lowerKey(x);
        Double higher = m_map.higherKey(x);
        // if (lower == null || higher == null) {
        // throw new IllegalArgumentException("X value is out of bounds");
        // }
        if (lower == null) {
            return m_map.get(m_map.firstKey());
        }
        if (higher == null) {
            return m_map.get(m_map.lastKey());
        }
        double y1 = m_map.get(lower);
        double y2 = m_map.get(higher);
        double x1 = lower;
        double x2 = higher;
        return y1 + (y2 - y1) / (x2 - x1) * (x - x1);
    }
}
