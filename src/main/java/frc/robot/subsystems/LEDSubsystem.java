package frc.robot.subsystems;

// imports

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

/* 
in case we decide to make our own constants
import frc.robot.Constants.LEDConstants;
*/


public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private static final int PWM_PORT = 0;     // change to port number LEDs are connected to 
    private static final int LED_LENGTH = 60;  // change depending on total LED count

    private int brightness = 255;

    private int defaultR = 0;
    private int defaultG = 0;
    private int defaultB = 255; // set default color to blue

    private LEDPattern currentPattern = LEDPattern.SOLID;
    private int solidR = defaultR;
    private int solidG = defaultG;
    private int solidB = defaultB;

    // three different states for LEDs, can add more if needed
    public enum LEDPattern {
        SOLID,
        BLINK,
        RAINBOW
    }

    // constructor
    public LEDSubsystem() {
        led = new AddressableLED(PWM_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        setLedDefault();
    }

 // methods

    public int getNumLEDS() {
        return buffer.getLength();
    }

    // sets all of the LEDs to the same RGB color
    public void setAllLedsRGB(int r, int g, int b) {
        currentPattern = LEDPattern.SOLID;
        solidR = r;
        solidG = g;
        solidB = b;
        setSolidColor();
    }

    // sets RGB color for an individual LED
    public void setLedRGB(int index, int r, int g, int b) {
        if (index < 0 || index >= buffer.getLength()) return;

        buffer.setRGB(index,
                scaleBrightness(r),
                scaleBrightness(g),
                scaleBrightness(b));
    }

    public void setLedBrightness(int brightness) {
        this.brightness = Math.max(0, Math.min(255, brightness));
    }

    public void setLedPattern(LEDPattern pattern) {
        currentPattern = pattern;
    }

    public void setLedDefault() {
        setAllLedsRGB(defaultR, defaultG, defaultB);
    }

    // updates the LEDs depending on the current pattern selected
    private void updatePattern() {
        switch (currentPattern) {

            case SOLID:
                setSolidColor();
                break;
            // toggles LEDs on and off twice per second using timestamp to create blink pattern
            case BLINK:
                boolean on = ((int)(Timer.getFPGATimestamp() * 2)) % 2 == 0;

                for (int i = 0; i < buffer.getLength(); i++) {
                    if (on) {
                        buffer.setRGB(i,
                                scaleBrightness(solidR),
                                scaleBrightness(solidG),
                                scaleBrightness(solidB));
                    } else {
                        buffer.setRGB(i, 0, 0, 0);
                    }
                }
                break;
            // shifts hue over time to create rainbow pattern
            case RAINBOW:
                double hueShift = (Timer.getFPGATimestamp() * 50) % 180;
                for (int i = 0; i < buffer.getLength(); i++) {
                    int hue = (int)((i * 180 / buffer.getLength() + hueShift) % 180);
                    buffer.setHSV(i, hue, 255, brightness);
                }
                break;
        }
    }

    private void setSolidColor() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i,
                    scaleBrightness(solidR),
                    scaleBrightness(solidG),
                    scaleBrightness(solidB));
        }
    }

    // scales an rgb value based on current brightness
    private int scaleBrightness(int value) {
        return (value * brightness) / 255;
    }

 // commands

    public Command ledDefaultCommand() {
        return run(() -> setLedDefault());
    }

    public Command setAllLedsRGBCommand(int r, int g, int b) {
        return run(() -> setAllLedsRGB(r, g, b));
    }

    // pre shoot, set to a yellow blink display
    public Command ledPreShootCommand() {
        return run(() -> {
            setAllLedsRGB(255, 200, 0);
            setLedPattern(LEDPattern.BLINK);
        });
    }

    // shooting, set to a solid green display when in shooting range
    public Command ledShootCommand(BooleanSupplier inRange) {
        return run(() -> {
            if (inRange.getAsBoolean()) {
                setAllLedsRGB(0, 255, 0);
            }
        }).until(() -> !inRange.getAsBoolean())
          .finallyDo((interrupted) -> setLedDefault());
    }

    // post shoot, set to a rainbow display
    public Command ledPostShootCommand() {
        return runOnce(() -> setLedPattern(LEDPattern.RAINBOW))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(runOnce(() -> setLedDefault()));
    }

    // updates pattern and sends data to LED strip
    @Override
    public void periodic() {
        updatePattern();
        led.setData(buffer);
    }
}