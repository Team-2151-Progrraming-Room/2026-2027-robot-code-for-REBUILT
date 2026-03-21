package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GoToHub extends SubsystemBase {

  Pose2d targetPoseToHub = new Pose2d(2.68, 3.666, Rotation2d.fromDegrees(-60));

  // Create the constraints to use while pathfinding
  PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  public Command GoToHubCommand() {
    return runOnce(
        () -> {
          AutoBuilder.pathfindToPose(
              targetPoseToHub, constraints, 0.0 // Goal end velocity in meters/sec
              );
        });
  }
}
