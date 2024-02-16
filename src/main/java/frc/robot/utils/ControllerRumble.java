package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ControllerRumble {
    public static final ConfigurableParameter<Double> kSmallRumbleIntensity = new ConfigurableParameter<Double>(0.5, "Small intensity Of The Rumble");
    public static final ConfigurableParameter<Double> kLargeRumbleIntensity = new ConfigurableParameter<Double>(1.0, "Large intensity Of The Rumble");
    public static final ConfigurableParameter<Double> kShortTime = new ConfigurableParameter<Double>(0.1, "This is the time the short rumble takes");
    public static final ConfigurableParameter<Double> kLongTime = new ConfigurableParameter<Double>(0.5, "This is the time the long rumble takes");

    Timer timer = new Timer();

    public void updatePeriodic() {
        if (timer.hasElapsed(kShortTime.get())) {
            driverSmallShort();
            timer.reset();

        }

        if (timer.hasElapsed(kLongTime.get())) {
            driverSmallLong();
            timer.reset();
        }

        if (timer.hasElapsed(kShortTime.get())) {
            driverBigShort();
            timer.reset();
        }

        if (timer.hasElapsed(kLongTime.get())) {
            driverBigLong();
            timer.reset();
        }

        if (timer.hasElapsed(kShortTime.get())) {
            operatorBigShort();
            timer.reset();
            
        }

        if (timer.hasElapsed(kLongTime.get())) {
            operatorSmallShort();
            timer.reset();
        }


        if (timer.hasElapsed(kShortTime.get())) {
            operatorBigLong();
            timer.reset();
            
        }

        if (timer.hasElapsed(kLongTime.get())) {
            operatorBigShort();
            timer.reset();
        }
    }

    public void driverSmallShort() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.start();
    }

    public void driverSmallLong() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.start();
    }

    public void operatorSmallShort() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.start();
    }

    public void operatorSmallLong() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.start();
    }

    public void driverBigShort() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.start();
    }

    public void driverBigLong() {
        RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble, kSmallRumbleIntensity.get());
        timer.start();
    }

    public void operatorBigShort() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.start();
    }

    public void operatorBigLong() {
        RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, kLargeRumbleIntensity.get());
        timer.start();
    }
}
