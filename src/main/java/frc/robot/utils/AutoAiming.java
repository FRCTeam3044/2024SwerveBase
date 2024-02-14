package frc.robot.utils;

public class AutoAiming {
    public static final int degree = 3;

    private PolynomialRegression polynomialRegression;

    public AutoAiming() {
        polynomialRegression = new PolynomialRegression(null, null, degree);
    }
}
