package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.StateMachine.State;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_RainbowFirstPixelHue;
    private RobotContainer m_robotContainer;
    private Timer timer = new Timer();

    /**
     * Creates a new LEDSubsystem to control Addressable LEDs
     * 
     * @param port      The PWM Port of the LEDs
     * @param ledLength The number of LEDs on the strand
     * @param robotContainer The robot container
     */
    public LEDSubsystem(int port, int ledLength, RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
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
            m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        m_RainbowFirstPixelHue += 3;
        m_RainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
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
        int pixel = (cameraPixel * m_ledBuffer.getLength()) / 1280;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0,0,0);
        }

        m_ledBuffer.setRGB(pixel, changeBrightness(255), changeBrightness(165), 0);
        m_ledBuffer.setRGB(pixel + 1, changeBrightness(255), changeBrightness(165), 0);
        m_ledBuffer.setRGB(pixel - 1, changeBrightness(255), changeBrightness(165), 0);

        m_led.setData(m_ledBuffer);
    }

    private int changeBrightness(int input) {
        double before = input / LEDConstants.LEDBrightnessModifier.get();
        int output = (int) Math.round(before);
        return output;
    }
    @Override
    public void periodic() {
        // setRainbow();
        if(m_robotContainer.stateMachineCommand.isScheduled()) {
            State currentState = RobotContainer.stateMachine.currentState;
            switch (currentState) {
                case TARGETING_NOTE: 
                    setCompass(RobotContainer.m_noteDetection.midpoint);
                case NOTE_LOADED:
                    setColor(0,255,0);
                case READY_TO_SHOOT:
                    blinkColor(0,255,0, 0.5);
                case NO_NOTE:
                    setRainbow();
            }
        } else {
            setRainbow();
        }
    }
}
