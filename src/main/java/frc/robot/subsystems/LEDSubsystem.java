package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_topRainbowFirstPixelHue;
    private int m_sideRainbowFirstPixelHue;
    private int m_rainbowFirstPixelHue;
    private Boolean rainbow = true;

    /**
     * Creates a new LEDSubsystem to control Addressable LEDs
     * @param port The PWM Port of the LEDs
     * @param ledLength The number of LEDs on the strand
     */
    public LEDSubsystem(int port, int ledLength) {
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void changeColor(int r, int g, int b) {
        rainbow = false;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setTopRainbow() {
        for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength + 1; i++) {
          final var hue = (m_topRainbowFirstPixelHue + (i * 180 / LEDConstants.topLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_topRainbowFirstPixelHue += 3;
        m_topRainbowFirstPixelHue %= 180;
    }

    public void setSidesRainbow() {
        for (var i = 0; i < LEDConstants.sideLEDLength; i++) {
          final var hue = (m_sideRainbowFirstPixelHue + (i * 180 / LEDConstants.sideLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_sideRainbowFirstPixelHue += 3;
        m_sideRainbowFirstPixelHue %= 180;
    }

    public void setRainbow() {
        for (var i = 0; i < m_ledBuffer.getLength() + 1; i++) {
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / LEDConstants.topLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
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
    }

    public void setCompass(int cameraPixel) {
        int pixel = (cameraPixel * LEDConstants.topLEDLength) / 1280;

        m_ledBuffer.setRGB(pixel, 255, 165, 0);
        m_ledBuffer.setRGB(pixel + 1, 255, 165, 0);
        m_ledBuffer.setRGB(pixel - 1, 255, 165, 0);
    }

    @Override
    public void periodic() {
        if(rainbow) {
            setTopRainbow();
            m_led.setData(m_ledBuffer);
        }
    }
}
