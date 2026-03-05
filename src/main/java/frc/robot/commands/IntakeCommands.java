package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class IntakeCommands extends Command {
  Intake intake;

  public Command intakeOffCommand() {
    return Commands.sequence(intake.IntakeOffCommand());
  }

  public Command intakeOnCommand() {
    return Commands.sequence(intake.IntakeOnCommand());
  }

  public Command intakeReverseCommand() {
    return Commands.sequence(intake.IntakeReverseCommand());
  }
}
