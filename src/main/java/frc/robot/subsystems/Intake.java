package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusName;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final TalonFXS m_IntakeMotor =
      new TalonFXS(IntakeConstants.kIntakeMotor, CanbusName.rioCANBus);
  private final TalonFXSConfiguration configs = new TalonFXSConfiguration();
  private final CurrentLimitsConfigs IntakeLimitConfigs = new CurrentLimitsConfigs();

  public Intake() {
    m_IntakeMotor.stopMotor();

    // Current Limit Configs
    IntakeLimitConfigs.withStatorCurrentLimit(IntakeConstants.kIntakeStatorCurrentLimit);
    IntakeLimitConfigs.withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit);

    // Config applications
    configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    configs.withCurrentLimits(
        IntakeLimitConfigs); // Current have this commented out so that the temp current limits
    // don't get applied

    m_IntakeMotor.getConfigurator().apply(configs);
  }

  // methods to turn motor on/off
  public void IntakeMotorOn() {
    m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  public void IntakeMotorOff() {
    double m_IntakeRpmTarget = 0.0;

    m_IntakeMotor.set(m_IntakeRpmTarget);
  }

  // Commands
  public Command IntakeMotorOnCommand() {
    return runOnce(
        () -> {
          IntakeMotorOn();
        });
  }

  public Command IntakeMotorOffCommand() {
    return runOnce(
        () -> {
          IntakeMotorOff();
        });
  }

  public Command IntakeMotorIntakeCommand() {
    return run(
        () -> {
          m_IntakeMotor.set(IntakeConstants.kIntakeIntakeSpeed);
        });
  }

  public Command IntakeDefaultIntakeCommand() {
    return run(
        () -> {
          m_IntakeMotor.set(IntakeConstants.kIntakeDefaultSpeed);
        });
  }

  public Command IntakeMotorOnSequenceCommand() {
    return Commands.sequence(
        IntakeMotorOnCommand(),
        Commands.waitSeconds(IntakeConstants.kIntakeOutputTime),
        IntakeMotorOffCommand());
  }

  public Command IntakeIntakeSequenceCommand() {
    return Commands.sequence(
        IntakeMotorIntakeCommand(),
        Commands.waitSeconds(IntakeConstants.kIntakeIntakeTime),
        IntakeMotorOffCommand());
  }
}
