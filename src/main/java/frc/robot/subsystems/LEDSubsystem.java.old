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
    private int m_topRainbowFirstPixelHue;
    private int m_sideRainbowFirstPixelHue;
    private int m_rainbowFirstPixelHue;
    private RobotContainer m_robotContainer;
    private Timer timer = new Timer();
    private int purpleGoldCenterLed = 1;
    /**
     * Creates a new LEDSubsystem to control Addressable LEDs
     * @param port The PWM Port of the LEDs
     * @param ledLength The number of LEDs on the strand
     */
    public LEDSubsystem(int port, int ledLength, RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(104);
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

    public void setTopRainbow() {
        for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength; i++) {
          final var hue = (m_topRainbowFirstPixelHue + (i * 180 / LEDConstants.topLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        m_topRainbowFirstPixelHue += 3;
        m_topRainbowFirstPixelHue %= 180;
    }

    public void setSidesRainbow() {
        for (var i = 0; i < LEDConstants.sideLEDLength; i++) {
          final var hue = (m_sideRainbowFirstPixelHue + (i * 180 / LEDConstants.sideLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        for (var i = m_ledBuffer.getLength(); i < LEDConstants.sideLEDLength + LEDConstants.topLEDLength; i--) {
            final var hue = (m_sideRainbowFirstPixelHue + (i * 180 / LEDConstants.sideLEDLength)) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        m_sideRainbowFirstPixelHue += 3;
        m_sideRainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }
     
    public void setRainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;

        m_led.setData(m_ledBuffer);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setColorTop(int r, int g, int b) {
        for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength + 1; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setColorSides(int r, int g, int b) {
        for (var i = 0; i < LEDConstants.sideLEDLength; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        for (var i = LEDConstants.sideLEDLength + LEDConstants.topLEDLength; i < LEDConstants.sideLEDLength + LEDConstants.topLEDLength + LEDConstants.sideLEDLength; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void blinkColorTop(int r, int g, int b, double time) {
        if(timer.get() == 0) {
            timer.start();
        } else if(timer.get() >= time) {
            timer.reset();
        }
        if(timer.get() < time / 2) {
            for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength + 1; i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
            m_led.setData(m_ledBuffer);
        } else {
            for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength + 1; i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }

    public void setCompass(int cameraPixel) {
        int pixel = (cameraPixel * LEDConstants.topLEDLength) / 1280;

        m_ledBuffer.setRGB(pixel, 255, 165, 0);
        m_ledBuffer.setRGB(pixel + 1, 255, 165, 0);
        m_ledBuffer.setRGB(pixel - 1, 255, 165, 0);
        m_led.setData(m_ledBuffer);
    }

    public void setSidesPurpleGold() {
        for (var i = 0; i < LEDConstants.sideLEDLength; i++) {
            m_ledBuffer.setRGB(i, 128, 0, 128);
        }
        for (var i = LEDConstants.sideLEDLength + LEDConstants.topLEDLength; i < LEDConstants.sideLEDLength + LEDConstants.topLEDLength + LEDConstants.sideLEDLength; i++) {
            m_ledBuffer.setRGB(i, 128, 0, 128);
        }
        m_ledBuffer.setRGB(purpleGoldCenterLed - 1, 255, 215, 0);
        m_ledBuffer.setRGB(purpleGoldCenterLed, 255, 215, 0);
        m_ledBuffer.setRGB(purpleGoldCenterLed + 1, 255, 215, 0);
        m_led.setData(m_ledBuffer);
        if(purpleGoldCenterLed == LEDConstants.sideLEDLength) {
            purpleGoldCenterLed = 1;
        } else {
            purpleGoldCenterLed++;
        }
    }

    @Override
    public void periodic() {
        // setSidesRainbow();
        // if(m_robotContainer.stateMachineCommand.isScheduled()) {
        //     setSidesRainbow();
        // } else {
        //     setSidesRainbow();
        // }

        // State state = RobotContainer.stateMachine.getState();
        // if(state == State.NOTE_LOADED) {
        //     setColorTop(255, 165, 0);
        // } else if(state == State.OWNS_NOTE) {
        //     blinkColorTop(255,165,0, 0.250);
        // } else if(state == State.TARGETING_NOTE) {
        //     if(RobotContainer.m_noteDetection.hasNote) {
        //         setCompass(RobotContainer.m_noteDetection.getClosestNoteCameraXPosition());
        //     }
        // } else if(state == State.READY_TO_SHOOT) {
        //     setColorTop(0, 255, 0);
        // }
    }
}
