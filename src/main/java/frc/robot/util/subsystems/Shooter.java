package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // all the krakens and minion we will be using
  private final TalonFX ShooterMotor1 = new TalonFX(ShooterConstants.KshooterMotor);
  private final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.KshooterMotor2);
  private final TalonFX ShooterMotor3 = new TalonFX(ShooterConstants.KshooterMotor3);
  private final TalonFX ShooterMotor4 = new TalonFX(ShooterConstants.KshooterMotor4);
  private final TalonFX ShooterMotor5 = new TalonFX(ShooterConstants.KshooterMotor5);
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
  }

  public void stopMotors() {
    ShooterMotor1.stopMotor();
    ShooterMotor2.stopMotor();
    ShooterMotor3.stopMotor();
    ShooterMotor4.stopMotor();
    ShooterMotor5.stopMotor();
    ShooterMotor6.stopMotor();
  }
}
