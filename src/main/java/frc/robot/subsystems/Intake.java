package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final TalonFXS m_IntakeMotor = new TalonFXS(IntakeConstants.kIntakeMotor);
  private final TalonFXSConfiguration configs = new TalonFXSConfiguration();
  private final CurrentLimitsConfigs IntakeLimitConfigs = new CurrentLimitsConfigs();
  Color IntakeStatus = new Color(255, 0, 0);
  private final TalonFXSConfiguration VelocityControl1 = new TalonFXSConfiguration();

  public Intake() {
    Color IntakeStatus = new Color(255, 0, 0);
    SmartDashboard.putString("IntakeStatus", IntakeStatus.toHexString());
    m_IntakeMotor.stopMotor();

    // Current Limit Configs
    IntakeLimitConfigs.withStatorCurrentLimit(IntakeConstants.kIntakeStatorCurrentLimit);
    IntakeLimitConfigs.withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit);

    // Config applications
    configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    VelocityControl1.withCurrentLimits(IntakeLimitConfigs);

    VelocityControl1.Slot0.kS = 0; // Add 0.1 V output to overcome static friction
    VelocityControl1.Slot0.kV = 1; // A velocity target of 1 rps results in 0.5 V output
    VelocityControl1.Slot0.kP = 1; // An error of 1 rps results in 0.5 V output
    VelocityControl1.Slot0.kI = 0; // no output for integrated error
    VelocityControl1.Slot0.kD = 0; // no output for error derivative

    configs.withCurrentLimits(
        IntakeLimitConfigs); // Current have this commented out so that the temp current limits
    // don't get applied

    m_IntakeMotor.getConfigurator().apply(VelocityControl1);
  }

  // methods to turn motor on
  public void IntakeOn() {
    IntakeStatus = new Color(0, 255, 0);
    SmartDashboard.putString("IntakeStatus", IntakeStatus.toHexString());
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    m_IntakeMotor.setControl(m_request.withVelocity(-20).withFeedForward(0.2));
  }

  public void IntakeIdle() {
    IntakeStatus = new Color(255, 0, 0);
    SmartDashboard.putString("IntakeStatus", IntakeStatus.toHexString());
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    m_IntakeMotor.setControl(m_request.withVelocity(-7).withFeedForward(0.2));
  }

  public void IntakeOff() {
    IntakeStatus = new Color(255, 0, 0);
    SmartDashboard.putString("IntakeStatus", IntakeStatus.toHexString());
    m_IntakeMotor.stopMotor();
  }

  public void IntakeReverse() {
    IntakeStatus = new Color(255, 255, 0);
    SmartDashboard.putString("IntakeStatus", IntakeStatus.toHexString());
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    m_IntakeMotor.setControl(m_request.withVelocity(20).withFeedForward(0.2));
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

  public Command IntakeIdleCommand() {
    return run(
        () -> {
          IntakeIdle();
        });
  }
}
