package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
  private final TalonFX ShooterMotor1 = new TalonFX(ShooterConstants.KshooterMotor);
  private final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.KshooterMotor2);
  private final TalonFX ShooterMotor3 = new TalonFX(ShooterConstants.KshooterMotor3);
  private final TalonFX ShooterMotor4 = new TalonFX(ShooterConstants.KshooterMotor4);
  // motor that moves the hood up and down
  XboxController controller = new XboxController(0);

  public double Hoodposition = 0.0;

  private final TalonFX ShooterMotor5 = new TalonFX(ShooterConstants.KshooterMotor5);

  // minion motor/intake motor
  private final TalonFXS ShooterMotor6 = new TalonFXS(ShooterConstants.KshooterMotor6);
  private final TalonFXS Indexer = new TalonFXS(ShooterConstants.KindexerMotor);
  private final CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

  // configurations
  private final TalonFXConfiguration hey = new TalonFXConfiguration();
  private final TalonFXSConfiguration hey2 = new TalonFXSConfiguration();
  public double shootSpeed = 0.5;
  public int mode = 1;

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
    Indexer.getConfigurator().apply(hey2);
  }

  public void ZeroHood() {}

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

  public void hoodStop() {
    ShooterMotor5.stopMotor();
  }

  public void hoodDown() {
    ShooterMotor5.set(-0.2);
  }

  public void shootMode() {
    SmartDashboard.putString("Shooter Mode", "Shoot Mode");
    ShooterMotor1.set(shootSpeed);
    ShooterMotor2.set(shootSpeed);
    ShooterMotor3.set(shootSpeed);
    ShooterMotor4.set(shootSpeed);

    ShooterMotor6.set(0.4);
    Runnable task =
        new Runnable() {
          @Override
          public void run() {
            System.out.println("The task is executed after a 0.3 second delay!");
            // Add your specific task logic here
            Indexer.set(0.4);
          }
        };
    // Create a ScheduledExecutorService
    ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);

    // Schedule the task to run once after a 300 millisecond delay
    executor.schedule(task, 300, TimeUnit.MILLISECONDS);

    // Important: You should eventually shut down the executor service
    // to allow the application to exit gracefully
    executor.shutdown();
  }

  public void passMode() {
    SmartDashboard.putString("Shooter Mode", "Pass Mode");
    mode = mode * -1;
    if (mode == 1) {
      shootSpeed = 0.5;

    } else if (mode == -1) {
      shootSpeed = 0.7;
    }
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
