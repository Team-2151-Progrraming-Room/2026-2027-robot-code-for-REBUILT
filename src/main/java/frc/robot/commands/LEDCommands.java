package frc.robot.commands;

// IMPORTS //

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

public class LEDCommands {

  LEDSubsystem leds;

  public LEDCommands(LEDSubsystem leds) {
    this.leds = leds;
  }

  public Command ledDefaultCommand() {
    return leds.ledDefaultCommand();
  }

  public Command setAllLedsRGBCommand(int r, int g, int b) {
    return leds.setAllLedsRGBCommand(r, g, b);
  }

  public Command ledPreShootCommand() {
    return leds.ledPreShootCommand();
  }

  public Command ledShootCommand(BooleanSupplier inRange) {
    return leds.ledShootCommand(inRange);
  }

  public Command ledActiveIntakingCommand() {
    return leds.ledActiveIntakingCommand();
  }

  public Command ledActiveSnowblowingCommand() {
    return leds.ledActiveSnowblowingCommand();
  }

  public Command ledBootupCommand() {
    return leds.ledBootupCommand();
  }
}
