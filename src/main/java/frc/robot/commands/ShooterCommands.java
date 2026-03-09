package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

public class ShooterCommands extends Command {

  Shooter shooter;

  public Command shootCommand() {
    return Commands.sequence(shooter.shootModeCommand());
  }

  public Command stopShooterCommand() {
    return Commands.sequence(shooter.stopShooterCommand());
  }

  public Command passModeCommand() {
    return Commands.sequence(shooter.passModeCommand());
  }

  public Command hoodupCommand() {
    return Commands.sequence(shooter.hoodUpCommand());
  }

  public Command hooddownCommand() {
    return Commands.sequence(shooter.hoodDownCommand());
  }

  public Command hoodStopCommand() {
    return Commands.sequence(shooter.hoodStopCommand());
  }
}
