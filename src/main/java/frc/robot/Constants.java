// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class Vision {
    public static final String FrontRightCamera = "FrontRightCamera";
    public static final String FrontLeftCamera = "FrontLeftCamera";
    public static final String BackRightCamera = "BackRightCamera";
    public static final String BackLeftCamera = "BackLeftCamera";

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToFrontRightCamera =
        new Transform3d(
            new Translation3d(0.2921, -0.31242, 0.127), new Rotation3d(0, 0.5235987756, 1.571));
    public static final Transform3d kRobotToFrontLeftCamera =
        new Transform3d(
            new Translation3d(0.2921, 0.31242, 0.127), new Rotation3d(0, 0.5235987756, 0));
    public static final Transform3d kRobotToBackRightCamera =
        new Transform3d(
            new Translation3d(-0.254, -0.3048, 0.127), new Rotation3d(0, 0.5235987756, 3.1415926));
    public static final Transform3d kRobotToBackLeftCamera =
        new Transform3d(
            new Translation3d(-0.254, 0.3175, 0.127), new Rotation3d(0, 0.5235987756, 4.71239));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class IntakeConstants {
    public static final int kIntakeMotor = 14;

    public static final int kIntakeStatorCurrentLimit = 60;
    public static final int kIntakeSupplyCurrentLimit = 60;
  }

  public static class ShooterConstants {

    public static final int kShooterTopFront = 27;
    public static final int kShooterTopRear = 29;
    public static final int kShooterBottomFront = 20;
    public static final int kShooterBottomRear = 21;
    public static final int kHood = 18;
    public static final int kFeeder = 35;
    public static final int KindexerMotor = 24;

    public static final int KshooterStatorCurrentLimit = 10;
    public static final int kshooterSupplyCurrentLimit = 10;
    public static final double KshooterSpeed1 = 0.5;
    public static final double KshooterSpeed2 = 0.5;
    public static final double KshooterSpeed3 = 0.5;
    public static final double KshooterSpeed4 = 0.5;
    public static final double KshooterSpeed5 = 0.5;
    public static final double KshooterSpeed6 = 0.5;

    public static final double KshooterSpeed1pass = 0.7;
    public static final double KshooterSpeed2pass = 0.7;
    public static final double KshooterSpeed3pass = 0.7;
    public static final double KshooterSpeed4pass = 0.7;
    public static final double KshooterSpeed5pass = 0.7;
    public static final double KshooterSpeed6pass = 0.7;
  }

  public static class CanbusName {
    public static final String rioCANBus = "rio";
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
