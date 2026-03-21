package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.WPINetJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // all the krakens and minion we will be using
  private final TalonFX kShooterTopFront = new TalonFX(ShooterConstants.kShooterTopFront);
  private final TalonFX kShooterTopRear = new TalonFX(ShooterConstants.kShooterTopRear);
  private final TalonFX kShooterBottomFront = new TalonFX(ShooterConstants.kShooterBottomFront);
  private final TalonFX kShooterBottomRear = new TalonFX(ShooterConstants.kShooterBottomRear);
  // motor that moves the hood up and down
  XboxController controller = new XboxController(0);

  private final TalonFX kHood = new TalonFX(ShooterConstants.kHood);

  // minion motor for feeder and indexer
  private final TalonFXS kFeeder = new TalonFXS(ShooterConstants.kFeeder);
  private final TalonFXS Indexer = new TalonFXS(ShooterConstants.KindexerMotor);
  private final CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

  // configurations
  private final TalonFXConfiguration hey = new TalonFXConfiguration();
  private final TalonFXSConfiguration hey2 = new TalonFXSConfiguration();
  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  public double shootSpeed = 0.7;

  //Boolean for Passing and Shooting Mode
  public boolean shootingOrPassing = true;
  
  public Shooter() {
    // Stop motors
    kShooterTopFront.stopMotor();
    kShooterTopRear.stopMotor();
    kShooterBottomFront.stopMotor();
    kShooterBottomRear.stopMotor();
    kHood.stopMotor();
    kFeeder.stopMotor();

    // Config Current limit
    configs.withSupplyCurrentLimit(10);
    configs.withStatorCurrentLimit(10);

    // Minion configs with current limits
    hey2.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    hey.withCurrentLimits(configs);
    hey2.withCurrentLimits(configs);

    // Apply TalonFX Configuration
    kShooterTopFront.getConfigurator().apply(hey);
    kShooterTopRear.getConfigurator().apply(hey);
    kShooterBottomFront.getConfigurator().apply(hey);
    kShooterBottomRear.getConfigurator().apply(hey);

    // Apply TalonFXS Configuration
    kFeeder.getConfigurator().apply(hey2);
    Indexer.getConfigurator().apply(hey2);

    // PID Configs
    hoodConfig.Slot0.kP = 0;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;

    // Apply PID Configs
    kHood.getConfigurator().apply(hoodConfig);
  }

  public void stopShooter() {
    kShooterTopFront.stopMotor();
    kShooterTopRear.stopMotor();
    kShooterBottomFront.stopMotor();
    kShooterBottomRear.stopMotor();
    kFeeder.stopMotor();
    Indexer.stopMotor();
  }

  public void stopIntakeShooter() {
    kFeeder.stopMotor();
  }

  public void hoodShootingPosition() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    if (shootingOrPassing == false) {
      kFeeder.setControl(m_request.withPosition(0.1));
      shootingOrPassing = true;
    }
  }

  public void hoodPassingPosition() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    if (shootingOrPassing == true) {
      kFeeder.setControl(m_request.withPosition(0.1));
      shootingOrPassing = false;
    }
  }

  public void hoodStop() {
    kHood.stopMotor();
  }

  public void shootMode() {
    SmartDashboard.putString("Shooter Mode", "Shoot Mode");
    kShooterTopFront.set(-0.5);
    kShooterTopRear.set(0.5);
    kShooterBottomFront.set(-0.5);
    kShooterBottomRear.set(0.5);

    kFeeder.set(-0.4);
  }

  public void passMode() {
    SmartDashboard.putString("Shooter Mode", "Pass Mode");
    kShooterTopFront.set(-shootSpeed);
    kShooterTopRear.set(shootSpeed);
    kShooterBottomFront.set(-shootSpeed);
    kShooterBottomRear.set(shootSpeed);

    kFeeder.set(-0.4);
  }

  public void indexerOn() {
    Indexer.set(0.3);
  }

  public void indexerOff() {
    Indexer.set(0.3);
  }

  public double getkShooterTopFrontVelocity() {
    double velocity = kShooterTopFront.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getkShooterTopRearVelocity() {
    double velocity = kShooterTopRear.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getkShooterBottomFrontVelocity() {
    double velocity = kShooterBottomFront.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getkShooterBottomRearVelocity() {
    double velocity = kShooterBottomRear.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getkFeederVelocity() {
    double velocity = kFeeder.getVelocity().getValueAsDouble();
    return velocity;
  }

  public Boolean atShooterSpeed() {
    if (MathUtil.isNear(-0.5, getkShooterTopFrontVelocity(), 0.1)
        && MathUtil.isNear(0.5, getkShooterTopRearVelocity(), 0.1)
        && MathUtil.isNear(-0.5, getkShooterBottomFrontVelocity(), 0.1)
        && MathUtil.isNear(0.5, getkShooterBottomRearVelocity(), 0.1)
        && MathUtil.isNear(-0.4, getkFeederVelocity(), 0.1)) {
      return true;
    }
    return false;
  }

  public boolean atPassingSpeed() {
    if (MathUtil.isNear(-0.7, getkShooterTopFrontVelocity(), 0.1)
        && MathUtil.isNear(0.7, getkShooterTopRearVelocity(), 0.1)
        && MathUtil.isNear(-0.7, getkShooterBottomFrontVelocity(), 0.1)
        && MathUtil.isNear(0.7, getkShooterBottomRearVelocity(), 0.1)
        && MathUtil.isNear(-0.4, getkFeederVelocity(), 0.1)) {
      return true;
    }
    return false;
  }

  public Command IndexerOn() {
    return runOnce(
        () -> {
          indexerOn();
        });
  }

  public Command shootModeCommand() {

    return runOnce(
        () -> {
          shootMode();
        });
  }

  public Command HoodShootingCommand() {
    return runOnce(
        () -> {
          hoodShootingPosition();
        });
  }

  public Command HoodPassingCommand() {
    return runOnce(
        () -> {
          hoodPassingPosition();
        });
  }

  public Command passModeCommand() {

    return runOnce(
        () -> {
          passMode();
        });
  }

  public Command stopShooterCommand() {

    return runOnce(
        () -> {
          stopShooter();
        });
  }

  public Command hoodStopCommand() {

    return runOnce(
        () -> {
          hoodStop();
        });
  }
}
