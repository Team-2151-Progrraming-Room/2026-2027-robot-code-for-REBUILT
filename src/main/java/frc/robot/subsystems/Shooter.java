package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // all the krakens and minion we will be using
  private final TalonFX ShooterMotor1 = new TalonFX(ShooterConstants.KshooterMotor);
  private final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.KshooterMotor2);
  private final TalonFX ShooterMotor3 = new TalonFX(ShooterConstants.KshooterMotor3);
  private final TalonFX ShooterMotor4 = new TalonFX(ShooterConstants.KshooterMotor4);
  // motor that moves the hood up and down
  private final TalonFX ShooterMotor5 = new TalonFX(ShooterConstants.KshooterMotor5);
  // minion motor/intake motor
  private final TalonFXS ShooterMotor6 = new TalonFXS(ShooterConstants.KshooterMotor6);
  private final CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

  // configurations
  private final TalonFXConfiguration hey = new TalonFXConfiguration();
  private final TalonFXSConfiguration hey2 = new TalonFXSConfiguration();

  public Shooter() {
    ShooterMotor1.stopMotor();
    ShooterMotor2.stopMotor();
    ShooterMotor3.stopMotor();
    ShooterMotor4.stopMotor();
    ShooterMotor5.stopMotor();
    ShooterMotor6.stopMotor();
    configs.withSupplyCurrentLimit(10);
    configs.withStatorCurrentLimit(10);
    hey2.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    hey.withCurrentLimits(configs);
    hey2.withCurrentLimits(configs);
    ShooterMotor1.getConfigurator().apply(hey);
    ShooterMotor2.getConfigurator().apply(hey);
    ShooterMotor3.getConfigurator().apply(hey);
    ShooterMotor4.getConfigurator().apply(hey);
    ShooterMotor5.getConfigurator().apply(hey);
    ShooterMotor6.getConfigurator().apply(hey2);
  }

  public void stopShooter() {
    ShooterMotor1.stopMotor();
    ShooterMotor2.stopMotor();
    ShooterMotor3.stopMotor();
    ShooterMotor4.stopMotor();
  }

  public void stopIntakeShooter() {
    ShooterMotor6.stopMotor();
  }

  public void hoodUp() {

    ShooterMotor5.set(0.2);
  }

  public void hoodDown() {
    ShooterMotor5.set(-0.2);
  }

  public void shootMode() {

    ShooterMotor1.set(0.5);
    ShooterMotor2.set(0.5);
    ShooterMotor3.set(0.5);
    ShooterMotor4.set(0.5);

    ShooterMotor6.set(0.4);
  }

  public void passMode() {

    ShooterMotor1.set(0.7);
    ShooterMotor2.set(0.7);
    ShooterMotor3.set(0.7);
    ShooterMotor4.set(0.7);

    ShooterMotor6.set(0.4);
  }

  public Command shootModeCommand() {

    return runOnce(
        () -> {
          shootMode();
        });
  }

  public Command passModeCommand() {

    return runOnce(
        () -> {
          passMode();
        });
  }
}
