package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    private RobotContainer m_robotContainer;
    private Timer timer = new Timer();
    
    /**
     * Creates a new LEDSubsystem to control Addressable LEDs
     * @param port The PWM Port of the LEDs
     * @param ledLength The number of LEDs on the strand
     */
    public LEDSubsystem(int port, int ledLength, RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(52);
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
        for (var i = LEDConstants.sideLEDLength + 1; i < LEDConstants.topLEDLength; i++) {
          final var hue = (m_RainbowFirstPixelHue + (i * 180 / LEDConstants.topLEDLength)) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 64);
        }
        m_RainbowFirstPixelHue += 3;
        m_RainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < LEDConstants.topLEDLength; i++) {
            m_ledBuffer.setRGB(LEDConstants.LEDOffset + i, r, g, b);
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
            for (var i = 0; i < LEDConstants.topLEDLength; i++) {
                m_ledBuffer.setRGB(LEDConstants.LEDOffset + i, r, g, b);
            }
            m_led.setData(m_ledBuffer);
        } else {
            for (var i = 0; i < LEDConstants.topLEDLength; i++) {
                m_ledBuffer.setRGB(LEDConstants.LEDOffset + i, 0, 0, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }

    public void setCompass(int cameraPixel) {
        int pixel = (cameraPixel * LEDConstants.topLEDLength) / 1280;

        m_ledBuffer.setRGB(LEDConstants.LEDOffset + pixel, 255, 165, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + pixel + 1, 255, 165, 0);
        m_ledBuffer.setRGB(LEDConstants.LEDOffset + pixel - 1, 255, 165, 0);
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
        setRainbow();
        // setRainbow();
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
