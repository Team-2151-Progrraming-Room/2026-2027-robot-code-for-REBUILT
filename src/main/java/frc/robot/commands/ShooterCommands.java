package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import java.util.function.BooleanSupplier;

public class ShooterCommands extends Command {
  Shooter shootersystems;
  BooleanSupplier m_atShooterSpeed;
  BooleanSupplier m_atPassingSpeed;

  public ShooterCommands(
      Shooter shooter, BooleanSupplier atShooterSpeed, BooleanSupplier atPassingSpeed) {
    shootersystems = shooter;
    m_atShooterSpeed = atShooterSpeed;
    m_atPassingSpeed = atPassingSpeed;
  }

  public Command ShootingMode() {
    return Commands.sequence(
        Commands.parallel(shootersystems.shootModeCommand(), shootersystems.HoodShootingCommand()),
        Commands.waitSeconds(0.3),
        shootersystems.hoodStopCommand(),
        Commands.waitSeconds(0.5),
        shootersystems.IndexerOnCommand());
  }

  public Command PassingMode() {
    return Commands.sequence(
        Commands.parallel(shootersystems.passModeCommand(), shootersystems.HoodPassingCommand()),
        Commands.waitSeconds(1),
        shootersystems.IndexerOnCommand());
  }
}
