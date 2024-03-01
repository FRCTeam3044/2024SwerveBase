package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ControllerRumble {
    public static final ConfigurableParameter<Double> kSmallRumbleIntensity = new ConfigurableParameter<Double>(0.5, "Small intensity Of The Rumble");
    public static final ConfigurableParameter<Double> kLargeRumbleIntensity = new ConfigurableParameter<Double>(1.0, "Large intensity Of The Rumble");
    public static final ConfigurableParameter<Double> kShortTime = new ConfigurableParameter<Double>(0.25, "Short Rumble Time");
    public static final ConfigurableParameter<Double> kLongTime = new ConfigurableParameter<Double>(0.5, "Long Rumble Time");
    public static final ConfigurableParameter<Double> kWavePeriod = new ConfigurableParameter<>(2.0, "The period of the wave in seconds");

    private static double cancelTime;

    public static Timer timer = new Timer();
    private static Timer waveTimer = new Timer();
    private static Timer pulseTimer = new Timer();

    public static double XVariable = 0;

    public static double waveIntensity;

    public static double pulseTime;

    public static double currentTime;

    public static double pulseDutyCycle;

    public static double pulseIntensity;

    public static double pulseWave;

    public static double timeSeconds;

    private boolean driverRumbleActive = false;
    private boolean operatorRumbleActive = false;

    private static boolean hasInit = false;

    public static double pulsePeriod;

    public static void updatePeriodic() {
        if(!hasInit) {
            timer.start();
            waveTimer.start();
            // pulseTimer.start();
        }

        pulsePeriod = 1.0;
        pulseDutyCycle = 0.2;

        timeSeconds = 1;

        currentTime = timer.get();

        // pulseIntensity = (pulseTimer.get() % pulsePeriod) < (pulseDutyCycle * pulsePeriod) ? 1.0 : 0.0;


        SmartDashboard.putNumber("Pulse Intensity", pulseIntensity);

        // double waveIntensity = Math.sin(2 * Math.PI * currentTime / kWavePeriod.get());
        
        waveIntensity = (((1 + (Math.sin(2 * Math.PI * waveTimer.get() / 10))) * .5) * .5) + .5;
        pulseWave = Math.sin(2 * Math.PI * waveTimer.get() / 2);
        SmartDashboard.putNumber("Wave Intensity", waveIntensity);
        SmartDashboard.putNumber("Pulse Intensity", pulseIntensity);
        SmartDashboard.putNumber("Pulse Wave", pulseWave);
        // System.out.println(waveIntensity);

        if (timer.hasElapsed(cancelTime)) {
            RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
            waveIntensity = 0.0;
            timer.reset();
        }
        if( waveIntensity >= 2 /*waveTimer.get() >= 4 || waveIntensity <= 0 */) {
            waveTimer.restart();
            waveIntensity = 0;
        }

        // System.out.println(pulseIntensity);
        // if(pulseIntensity == 1) {
        //     pulseTimer.reset();
        // }

        if(pulseWave < 0) {
            pulseIntensity = 1;
        } 
        if (pulseWave > 0) {
            pulseIntensity = 0;
        }
    }

    public static void driverPulse(double intensity) {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble,  pulseIntensity);
    }

    public static void driverWave(double intensity) {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, intensity * waveIntensity);

        cancelTime = kLongTime.get();
    }

    public static void driverSmallShort() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kLeftRumble, kSmallRumbleIntensity.get());
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kRightRumble, -kSmallRumbleIntensity.get());
        timer.restart();

        cancelTime = kShortTime.get();
    }

    public static void driverSmallLong() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.restart();

        cancelTime = kLongTime.get();
    }

    public static void operatorSmallShort() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.restart();

        cancelTime = kShortTime.get();
    }

    public static void operatorSmallLong() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.restart();

        cancelTime = kLongTime.get();
    }

    public static void driverBigShort() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.restart();

        cancelTime = kShortTime.get();
    }

    public static void driverBigLong() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
        timer.restart();

        cancelTime = kLongTime.get();
    }

    public static void operatorBigShort() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.restart();

        cancelTime = kShortTime.get();
    }

    public static void operatorBigLong() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.restart();

        cancelTime = kLongTime.get();
    }
}
