package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ControllerRumble {
    public static final ConfigurableParameter<Double> kSmallRumbleIntensity = new ConfigurableParameter<Double>(0.4,
            "Small Rumble intensity");
    public static final ConfigurableParameter<Double> kLargeRumbleIntensity = new ConfigurableParameter<Double>(1.0,
            "Large Rumble intensity");
    public static final ConfigurableParameter<Double> kShortTime = new ConfigurableParameter<Double>(0.25,
            "Short Rumble Time");
    public static final ConfigurableParameter<Double> kLongTime = new ConfigurableParameter<Double>(0.5,
            "Long Rumble Time");
    public static final ConfigurableParameter<Double> kWavePeriod = new ConfigurableParameter<Double>(10.0, "Wave Period");
    public static ConfigurableParameter<Double> kPulsePeriod = new ConfigurableParameter<Double>(10.0, "Rumble Pulse Period");

    private static Timer driverRumbleTimer = new Timer();
    private static Timer operatorRumbleTimer = new Timer();

    private static boolean driverRumbleActive = false;
    private static boolean operatorRumbleActive = false;

    private static RumbleMode driverRumbleMode = RumbleMode.CONSTANT;
    private static RumbleMode operatorRumbleMode = RumbleMode.CONSTANT;

    private static double driverRumbleTime = 0;
    private static double operatorRumbleTime = 0;

    private static double driverRumbleIntensity = 0;
    private static double operatorRumbleIntensity = 0;

    public static void updatePeriodic() {
        if (driverRumbleActive) {
            if (driverRumbleTimer.hasElapsed(driverRumbleTime)) {
                RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
                driverRumbleActive = false;
                driverRumbleTimer.reset();
                driverRumbleTimer.stop();
                return;
            }
            if (driverRumbleMode == RumbleMode.WAVE) {
                double waveIntensity = (((1 + (Math.sin(driverRumbleTimer.get() * kWavePeriod.get()))) * .5) * .8) + .1;
                RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, waveIntensity);
                SmartDashboard.putNumber("Driver Wave Intensity", waveIntensity);
            } else if (driverRumbleMode == RumbleMode.PULSE){
                double pulseWave = Math.sin(driverRumbleTimer.get() * kPulsePeriod.get());
                double pulseIntensity = pulseWave > 0 ? driverRumbleIntensity : 0;
                RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, pulseIntensity);
                SmartDashboard.putNumber("Driver Pulse Intensity", pulseIntensity);
            }
        }

        if (operatorRumbleActive) {
            if (operatorRumbleTimer.hasElapsed(operatorRumbleTime)) {
                RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
                operatorRumbleActive = false;
                operatorRumbleTimer.reset();
                operatorRumbleTimer.stop();
                return;
            }
            if (operatorRumbleMode == RumbleMode.WAVE) {
                double waveIntensity = (((1 + (Math.sin(operatorRumbleTimer.get() * kWavePeriod.get()))) * .5) * .8) + .1;
                RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, waveIntensity);
                SmartDashboard.putNumber("Operator Wave Intensity", waveIntensity);
            } else if (operatorRumbleMode == RumbleMode.PULSE){
                double pulseWave = Math.sin(operatorRumbleTimer.get() * kPulsePeriod.get());
                double pulseIntensity = pulseWave > 0 ? operatorRumbleIntensity : 0;
                RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, pulseIntensity);
                SmartDashboard.putNumber("Operator Pulse Intensity", pulseIntensity);
            }
        }
    }

    public static void setControllerRumble(double rumbleTime, double rumbleIntensity, boolean driverController,
            RumbleMode rumbleMode) {
        if (driverController) {
            driverRumbleMode = rumbleMode;
            driverRumbleActive = true;
            driverRumbleTimer.reset();
            driverRumbleTimer.start();
            driverRumbleTime = rumbleTime;
            driverRumbleIntensity = rumbleIntensity;
            RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumbleIntensity);
        } else {
            operatorRumbleMode = rumbleMode;
            operatorRumbleActive = true;
            operatorRumbleTimer.reset();
            operatorRumbleTimer.start();
            operatorRumbleIntensity = rumbleIntensity;
            operatorRumbleTime = rumbleTime;
            RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, rumbleIntensity);
        }
    }

    public static void driverSmallShort() {
        setControllerRumble(kShortTime.get(), kSmallRumbleIntensity.get(), true, RumbleMode.CONSTANT);
    }

    public static void opSmallShort() {
        setControllerRumble(kShortTime.get(), kSmallRumbleIntensity.get(), false, RumbleMode.CONSTANT);
    }

    public static void driverbigShort() {
        setControllerRumble(kShortTime.get(), kLargeRumbleIntensity.get(), true, RumbleMode.CONSTANT);
    }

    public static void opBigShort() {
        setControllerRumble(kShortTime.get(), kLargeRumbleIntensity.get(), false, RumbleMode.CONSTANT);
    }

    public static void driverSmallLong() {
        setControllerRumble(kLongTime.get(), kSmallRumbleIntensity.get(), true, RumbleMode.CONSTANT);
    }

    public static void opSmallLong() {
        setControllerRumble(kLongTime.get(), kSmallRumbleIntensity.get(), false, RumbleMode.CONSTANT);
    }

    public static void driverbigLong() {
        setControllerRumble(kLongTime.get(), kLargeRumbleIntensity.get(), true, RumbleMode.CONSTANT);
    }

    public static void opBigLong() {
        setControllerRumble(kLongTime.get(), kLargeRumbleIntensity.get(), false, RumbleMode.CONSTANT);
    }

    public static void driverWaveLong(){
        setControllerRumble(kLongTime.get() * 5, kLargeRumbleIntensity.get(), true, RumbleMode.WAVE);
    }

    public static void opWaveLong(){
        setControllerRumble(kLongTime.get() * 5, kLargeRumbleIntensity.get(), true, RumbleMode.WAVE);
    }

    public static void drivePulseLong(){
        setControllerRumble(kLongTime.get() * 5, kLargeRumbleIntensity.get(), true, RumbleMode.PULSE);
    }

    public static void opPulseLong(){
        setControllerRumble(kLongTime.get() * 5, kLargeRumbleIntensity.get(), false, RumbleMode.PULSE);
    }
    
    public enum RumbleMode {
        CONSTANT,
        WAVE,
        PULSE
    }
}
