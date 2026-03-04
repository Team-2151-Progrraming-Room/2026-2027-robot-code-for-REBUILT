package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final TalonFXS m_IntakeMotor = new TalonFXS(IntakeConstants.kIntakeMotor);
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

  // methods to turn motor on
  public void IntakeOn() {
    m_IntakeMotor.set(-0.40);
  }

  public void IntakeOff() {
    m_IntakeMotor.set(0);
  }

  public void IntakeReverse() {
    m_IntakeMotor.set(0.4);
  }

  // Commands
  public Command IntakeOnCommand() {
    return runOnce(
        () -> {
          IntakeOn();
        });
  }

  public Command IntakeOffCommand() {
    return runOnce(
        () -> {
          IntakeOff();
        });
  }

  public Command IntakeReverseCommand() {
    return run(
        () -> {
          IntakeReverse();
        });
  }
}
