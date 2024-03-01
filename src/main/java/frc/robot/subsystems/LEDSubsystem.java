package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
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

    public void setRainbow() {
        for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength + 1; i++) {
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    @Override
    public void periodic() {
        if(rainbow) {
            setRainbow();
            m_led.setData(m_ledBuffer);
        }
    }

    public void defaultStatus() {
        rainbow = true;
    }
}
