package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
  // all the krakens and minion we will be using
  private final TalonFX kShooterTopFront = new TalonFX(ShooterConstants.kShooterTopFront);
  private final TalonFX kShooterTopRear = new TalonFX(ShooterConstants.kShooterTopRear);
  private final TalonFX kShooterBottomFront = new TalonFX(ShooterConstants.kShooterBottomFront);
  private final TalonFX kShooterBottomRear = new TalonFX(ShooterConstants.kShooterBottomRear);
  // motor that moves the hood up and down
  XboxController controller = new XboxController(0);

  public double Hoodposition = 0.0;

  private final TalonFX kHood = new TalonFX(ShooterConstants.kHood);

  // minion motor/intake motor
  private final TalonFXS kFeeder = new TalonFXS(ShooterConstants.kFeeder);
  private final TalonFXS Indexer = new TalonFXS(ShooterConstants.KindexerMotor);
  private final CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

  // configurations
  private final TalonFXConfiguration hey = new TalonFXConfiguration();
  private final TalonFXSConfiguration hey2 = new TalonFXSConfiguration();
  public double shootSpeed = 0.7;

  public Shooter() {

    kShooterTopFront.stopMotor();
    kShooterTopRear.stopMotor();
    kShooterBottomFront.stopMotor();
    kShooterBottomRear.stopMotor();
    kHood.stopMotor();
    kFeeder.stopMotor();
    configs.withSupplyCurrentLimit(10);
    configs.withStatorCurrentLimit(10);
    hey2.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    hey.withCurrentLimits(configs);
    hey2.withCurrentLimits(configs);
    kShooterTopFront.getConfigurator().apply(hey);
    kShooterTopRear.getConfigurator().apply(hey);
    kShooterBottomFront.getConfigurator().apply(hey);
    kShooterBottomRear.getConfigurator().apply(hey);
    kHood.getConfigurator().apply(hey);
    kFeeder.getConfigurator().apply(hey2);
    Indexer.getConfigurator().apply(hey2);
  }

  public void ZeroHood() {}

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

  public void hoodUp() {

    kHood.set(0.2);
  }

  public void hoodStop() {
    kHood.stopMotor();
  }

  public void hoodDown() {
    kHood.set(-0.2);
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

  public boolean atShooterSpeed() {
    if (MathUtil.isNear(-0.5, getkShooterTopFrontVelocity(), 0.1) && MathUtil.isNear(0.5, getkShooterTopRearVelocity(), 0.1) && MathUtil.isNear(-0.5, getkShooterBottomFrontVelocity(), 0.1) && MathUtil.isNear(0.5, getkShooterBottomRearVelocity(), 0.1) && MathUtil.isNear(-0.4, getkFeederVelocity(), 0.1)) {
      return true;
    }
    return false;
  }

  public boolean atPassingSpeed() {
    if (MathUtil.isNear(-0.7, getkShooterTopFrontVelocity(), 0.1) && MathUtil.isNear(0.7, getkShooterTopRearVelocity(), 0.1) && MathUtil.isNear(-0.7, getkShooterBottomFrontVelocity(), 0.1) && MathUtil.isNear(0.7, getkShooterBottomRearVelocity(), 0.1) && MathUtil.isNear(-0.4, getkFeederVelocity(), 0.1)) {
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

  public Command passModeCommand() {

    return runOnce(
        () -> {
          passMode();
        });
  }

  public Command hoodUpCommand() {

    return runOnce(
        () -> {
          hoodUp();
        });
  }

  public Command hoodDownCommand() {

    return runOnce(
        () -> {
          hoodDown();
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
