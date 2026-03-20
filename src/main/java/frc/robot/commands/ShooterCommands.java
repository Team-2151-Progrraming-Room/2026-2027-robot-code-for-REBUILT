package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Shooter;

import frc.robot.Constants.*;

public class ShooterCommands extends Command {
    Shooter shootersystems;
    BooleanSupplier m_atShooterSpeed;
    BooleanSupplier m_atPassingSpeed;

    public ShooterCommands(Shooter shooter, BooleanSupplier atShooterSpeed, BooleanSupplier atPassingSpeed) {
        shootersystems = shooter;
        m_atShooterSpeed = atShooterSpeed;
        m_atPassingSpeed = atPassingSpeed;
    }

    public Command ShootingMode() {
        return Commands.sequence(
            shootersystems.shootModeCommand(),
            Commands.waitUntil(m_atShooterSpeed),
            shootersystems.IndexerOn()
        );
    }

    public Command PassingMode() {
        return Commands.sequence(
            shootersystems.passModeCommand(),
            Commands.waitUntil(m_atPassingSpeed),
            shootersystems.IndexerOn()
        );
    }
}
