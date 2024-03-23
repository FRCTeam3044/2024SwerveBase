package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.StateMachine.State;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_RainbowFirstPixelHue;
    private int m_goldLedOffset;
    private Timer timer = new Timer();
    private RobotContainer m_robotContainer;
    private Debouncer m_detectionDebouncer = new Debouncer(0.15);

    /**
     * Creates a new LEDSubsystem to control Addressable LEDs
     * 
     * @param port           The PWM Port of the LEDs
     * @param ledLength      The number of LEDs on the strand
     * @param robotContainer The robot container
     */
    public LEDSubsystem(int port, int ledLength, RobotContainer robotContainer) {
        LEDConstants.LEDBrightnessModifier.get();
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        m_robotContainer = robotContainer;
    }

    public void changeColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setRainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_RainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 5);
        }
        m_RainbowFirstPixelHue += 3;
        m_RainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void setPurpleGold(boolean dim) {
        double secondOffset = (m_goldLedOffset + (Math.floor(m_ledBuffer.getLength() / 2))) % m_ledBuffer.getLength();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if (Math.abs(m_goldLedOffset - i) < 5 || Math.abs(secondOffset - i) < 5) {
                if (dim) {
                    m_ledBuffer.setRGB(i, changeBrightness(changeBrightness(127)),
                            changeBrightness(changeBrightness(110)), changeBrightness(changeBrightness(0)));
                } else {
                    m_ledBuffer.setRGB(i, changeBrightness(254), changeBrightness(220), changeBrightness(0));
                }
            } else {
                if (dim) {
                    m_ledBuffer.setRGB(i, changeBrightness(changeBrightness(54)),
                            changeBrightness(changeBrightness(19)), changeBrightness(changeBrightness(67)));
                } else {
                    m_ledBuffer.setRGB(i, changeBrightness(107), changeBrightness(38), changeBrightness(134));
                }
            }
        }
        m_goldLedOffset += 1;
        if (m_goldLedOffset > m_ledBuffer.getLength()) {
            m_goldLedOffset = 0;
        }
        m_led.setData(m_ledBuffer);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, changeBrightness(r), changeBrightness(g), changeBrightness(b));
        }
        m_led.setData(m_ledBuffer);
    }

    public void blinkColor(int r, int g, int b, double time) {
        r = changeBrightness(r);
        g = changeBrightness(g);
        b = changeBrightness(b);
        if (timer.get() == 0) {
            timer.start();
        } else if (timer.get() >= time) {
            timer.reset();
        }
        if (timer.get() < time / 2) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
            m_led.setData(m_ledBuffer);
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }

    public void setCompass(int cameraPixel) {
        int pixel = (int) (((double) cameraPixel * LEDConstants.LEDTopLength) / 1280);

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }

        m_ledBuffer.setRGB(LEDConstants.LEDOffset + LEDConstants.LEDTopLength - pixel, 255, 0, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + LEDConstants.LEDTopLength - pixel + 1, 255, 0, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + LEDConstants.LEDTopLength - pixel + 2, 255, 0, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + LEDConstants.LEDTopLength - pixel - 1, 255, 0, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + LEDConstants.LEDTopLength - pixel - 2, 255, 0, 0);

        m_led.setData(m_ledBuffer);
    }

    private int changeBrightness(int input) {
        double before = input / LEDConstants.LEDBrightnessModifier.get();
        int output = (int) Math.round(before);
        return output;
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled() || (LEDConstants.bypassEnabled.get() && !DriverStation.isFMSAttached())) {
            State currentState = RobotContainer.stateMachine.currentState;
            boolean shooterAligned = RobotContainer.elevator.elevatorAtAngle();
            boolean allowedToBlink = (DriverStation.isAutonomous() || (!DriverStation.isAutonomous()
                    && RobotContainer.m_driverController.getLeftTriggerAxis() > 0.5));
            SmartDashboard.putBoolean("Elevator Aligned", shooterAligned);
            switch (currentState) {

                case TARGETING_NOTE:
                    setCompass(RobotContainer.m_noteDetection.midpoint);
                    break;
                case NOTE_LOADED:
                    // TEmporary hack :D
                    if (RobotContainer.shooter.shooterAtSpeed()
                            && allowedToBlink && shooterAligned) {
                        blinkColor(0, 50, 0, 0.2);
                    } else {
                        setColor(254, 222, 0);
                    }
                    break;
                case READY_TO_SHOOT:
                    if (RobotContainer.shooter.shooterAtSpeed() && allowedToBlink && shooterAligned) {
                        blinkColor(0, 50, 0, 0.2);
                    } else {
                        setColor(254, 222, 0);
                    }
                    break;
                case NO_NOTE:
                    setPurpleGold(false);
                    break;
            }
        } else {
            if (m_detectionDebouncer.calculate(m_robotContainer.m_visionSubsystem.hasTargets())) {
                setColor(0, 50, 0);
            } else {
                if (LEDConstants.purpleGoldIdle.get()) {
                    setPurpleGold(true);
                } else {
                    setRainbow();
                }
            }
        }
    }
}
