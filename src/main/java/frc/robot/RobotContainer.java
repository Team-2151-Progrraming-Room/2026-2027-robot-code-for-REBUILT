// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsBLBR;
import frc.robot.subsystems.GoToHub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Touchboard.ActionButton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooterCommands = new Shooter();
  private final GoToHub goToHub = new GoToHub();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Booleans Supplier
  private final BooleanSupplier m_atShooterSpeed = () -> shooterCommands.atShooterSpeed();
  private final BooleanSupplier m_atPassingSpeed = () -> shooterCommands.atPassingSpeed();

  // Shooter Commands
  private final ShooterCommands shooterCommandsFiled =
      new ShooterCommands(shooterCommands, m_atShooterSpeed, m_atPassingSpeed);
  private final Command shootCommand = shooterCommandsFiled.ShootingMode();
  private final Command passCommand = shooterCommandsFiled.PassingMode();
  private final Command stopShooterCommand = shooterCommands.stopShooterCommand();

  // Go To Hub Command
  private final Command goToHubCommand = goToHub.GoToHubCommand();

  // Touchboard Buttons/Commands
  private final ActionButton ShootingMode = new ActionButton("ShootingMode", shootCommand);
  private final ActionButton PassingMode = new ActionButton("PassingMode", passCommand);
  private final ActionButton StopShooting =
      new ActionButton("StopShooting", shooterCommands.stopShooterCommand());
  private final ActionButton GoToHub = new ActionButton("GoToHub", goToHubCommand);
  private final ActionButton HoodShooting =
      new ActionButton("HoodShooting", shooterCommands.HoodShootingCommand());
  private final ActionButton HoodPassing =
      new ActionButton("HoodPassing", shooterCommands.HoodPassingCommand());
  private final ActionButton HoodStop =
      new ActionButton("HoodStop", shooterCommands.hoodStopCommand());
  private final ActionButton HoodStoep =
      new ActionButton("tee", shooterCommands.IndexerOnCommand());

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Intake Commands *luis is listening to top 10 ncs intro/outro songs while im writing ts*
  private final Intake Charlie = new Intake();
  private final Command George = Charlie.IntakeOffCommand();
  private final Command Jason = Charlie.IntakeOnCommand();
  private final Command Luis = Charlie.IntakeReverseCommand();

  // Touchboared docoammadnds
  // private final OneShotButton ShootingMode = new OneShotButton(ShootingMode, new *Command)
  // private final OneShotButton PassingMode = new OneShotButton(PassingMode, new *Command)

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstantsBLBR.BackLeft),
                new ModuleIOTalonFX(TunerConstantsBLBR.BackRight));

        vision = new Vision(drive::addVisionMeasurement);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstantsBLBR.BackLeft),
                new ModuleIOSim(TunerConstantsBLBR.BackRight));

        vision = new Vision(drive::addVisionMeasurement);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement);
        break;
    }

    NamedCommands.registerCommand("IntakeOn", Jason);
    NamedCommands.registerCommand("IntakeOff", George);
    NamedCommands.registerCommand("Shoot", shootCommand);
    NamedCommands.registerCommand("Pass", passCommand);
    NamedCommands.registerCommand("ShootSTOPPLEASEIBEG", stopShooterCommand);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.y().onTrue((George));
    controller.x().onTrue((Jason));
    controller.b().onTrue((Luis));

    controller.rightTrigger().whileTrue(shooterCommands.shootModeCommand());
    controller.rightTrigger().whileFalse(shooterCommands.stopShooterCommand());
    controller.rightBumper().whileTrue(shooterCommands.passModeCommand());
    if (controller.leftTrigger().getAsBoolean() == false
        && controller.leftBumper().getAsBoolean() == false) {
      shooterCommands.hoodStopCommand();
    }

    // Reset gyro to 0° when B button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
