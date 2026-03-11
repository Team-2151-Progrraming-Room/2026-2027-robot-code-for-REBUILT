package frc.robot.subsystems;

// IMPORTS //

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

/*
in case we decide to make our own constants
import frc.robot.Constants.LEDConstants;
*/

public class LEDSubsystem extends SubsystemBase {

  private final CANdle candle;
  private static final int CANDLE_ID = 0; // change this to the CAN ID of CANdle
  private static final int LED_LENGTH = 60; // change depending on total LED count

  private int brightness = 255;

  private int defaultR = 128;
  private int defaultG = 0;
  private int defaultB = 128; // set default color to purple

  private LEDPattern currentPattern = LEDPattern.SOLID;
  private int solidR = defaultR;
  private int solidG = defaultG;
  private int solidB = defaultB;

  // three different patterns for LEDs, can add more if needed
  public enum LEDPattern {
    SOLID,
    BLINK,
    PULSE
  }

  // constructor
  public LEDSubsystem() {

    candle = new CANdle(CANDLE_ID, "rio");

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 1.0;
    config.statusLedOffWhenActive = true;

    candle.configAllSettings(config, 100);

    setLedDefault();
  }

  // METHODS //

  // returns number of LEDs
  public int getNumLEDS() {
    return LED_LENGTH;
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
    if (index < 0 || index >= LED_LENGTH) return;

    candle.setLEDs(scaleBrightness(r), scaleBrightness(g), scaleBrightness(b), 0, index, 1);
  }

  // sets brightness for LEDs
  public void setLedBrightness(int brightness) {
    this.brightness = Math.max(0, Math.min(255, brightness));
    candle.configBrightnessScalar(brightness / 255.0, 0);
  }

  // sets the active pattern (RGB stays the same until next periodic)
  public void setLedPattern(LEDPattern pattern) {
    currentPattern = pattern;
  }

  // sets to default LED state (solid purple)
  public void setLedDefault() {
    setAllLedsRGB(defaultR, defaultG, defaultB);
  }

  // updates the LEDs depending on the current pattern selected
  // can be added onto/changed if more states are needed or different colors/patterns are wanted
  private void updatePattern() {

    switch (currentPattern) {

        // solid RGB color for LEDs
      case SOLID:
        setSolidColor();
        break;

        // toggles LEDs on and off twice per second using timestamp to create blink pattern
      case BLINK:
        boolean on = ((int) (Timer.getFPGATimestamp() * 2)) % 2 == 0;

        if (on) {
          candle.setLEDs(
              scaleBrightness(solidR),
              scaleBrightness(solidG),
              scaleBrightness(solidB),
              0,
              0,
              LED_LENGTH);
        } else {
          candle.setLEDs(0, 0, 0, 0, 0, LED_LENGTH);
        }

        break;

        // LED brightness level shifts between bright and dim
      case PULSE:
        double time = Timer.getFPGATimestamp();
        double wave = (Math.sin(time * 2) + 1) / 2.0;

        int pulseBrightness = (int) (wave * brightness);

        candle.setLEDs(
            (solidR * pulseBrightness) / 255,
            (solidG * pulseBrightness) / 255,
            (solidB * pulseBrightness) / 255,
            0,
            0,
            LED_LENGTH);

        break;
    }
  }

  // sets to a solid RGB color
  private void setSolidColor() {

    candle.setLEDs(
        scaleBrightness(solidR),
        scaleBrightness(solidG),
        scaleBrightness(solidB),
        0,
        0,
        LED_LENGTH);
  }

  // scales an RGB value based on current brightness
  private int scaleBrightness(int value) {
    return (value * brightness) / 255;
  }

  // COMMANDS //

  // default, solid blue display
  public Command ledDefaultCommand() {
    return run(() -> setLedDefault());
  }

  // sets all LEDs to same RGB
  public Command setAllLedsRGBCommand(int r, int g, int b) {
    return run(() -> setAllLedsRGB(r, g, b));
  }

  // pre shoot, set to a yellow blink display
  public Command ledPreShootCommand() {
    return run(
        () -> {
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
        })
        .until(() -> !inRange.getAsBoolean())
        .finallyDo((interrupted) -> setLedDefault());
  }

  // intaking, set to a solid blue display when intake is active
  public Command ledActiveIntakingCommand() {
    return run(
        () -> {
          setAllLedsRGB(0, 0, 255);
        });
  }

  // snowblowing, set to a solid white display when intake is snowblowing
  public Command ledActiveSnowblowingCommand() {
    return run(
        () -> {
          setAllLedsRGB(255, 255, 255);
        });
  }

  // bootup, set to a purple pulse display when robot is on but not in either auto or teleop modes
  public Command ledBootupCommand() {
    return run(
        () -> {
          solidR = 128;
          solidG = 0;
          solidB = 128;
          setLedPattern(LEDPattern.PULSE);
        });
  }

  // updates pattern
  @Override
  public void periodic() {
    updatePattern();
  }
}