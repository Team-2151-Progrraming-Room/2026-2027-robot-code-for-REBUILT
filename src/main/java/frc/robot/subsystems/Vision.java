package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {

    private static final double BASE_XY_STDDEV = 0.03;
    private static final double THETA_STDDEV = 9999.0; 

    private Matrix<N3, N1> computeVisionStdDevs(double distanceMeters, int numTags) {
    // XY error scales with distance² and improves proportionally with tag count
    double xyStdDev = BASE_XY_STDDEV * (distanceMeters * distanceMeters) / numTags;

    // Cap to prevent extreme values at long range from blowing up the filter.
    // Measurements beyond 7m or so should be rejected in Lesson 8 anyway.
    xyStdDev = Math.min(xyStdDev, 5.0);

    return VecBuilder.fill(xyStdDev, xyStdDev, THETA_STDDEV);
    }

    private final Drive m_drive;
    private final PhotonCamera m_camera = new PhotonCamera("FrontRightCamera");
    private final PhotonCamera m_camera2 = new PhotonCamera("FrontLeftCamera");
    private final PhotonCamera m_camera3 = new PhotonCamera("BackRightCamera");
    private final PhotonCamera m_camera4 = new PhotonCamera("BackLeftCamera");
    private final PhotonPoseEstimator m_poseEstimator;
    private final PhotonPoseEstimator m_poseEstimator2;
    private final PhotonPoseEstimator m_poseEstimator3;
    private final PhotonPoseEstimator m_poseEstimator4;
    private PhotonPipelineResult m_latestResult = new PhotonPipelineResult();
    private PhotonPipelineResult m_latestResult2 = new PhotonPipelineResult();
    private PhotonPipelineResult m_latestResult3 = new PhotonPipelineResult();
    private PhotonPipelineResult m_latestResult4 = new PhotonPipelineResult();

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

    public Vision(Drive drive) {
        m_drive = drive;
        AprilTagFieldLayout layout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        m_poseEstimator = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToFrontRightCamera);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimator2 = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToFrontLeftCamera);
        m_poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimator3 = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToBackRightCamera);
        m_poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        m_poseEstimator4 = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToBackLeftCamera);
        m_poseEstimator4.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        m_latestResult = m_camera.getLatestResult();
        Optional<EstimatedRobotPose> estimateOpt = Optional.empty();
        for (var result : m_camera.getAllUnreadResults()) {
            estimateOpt = m_poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimateOpt.isEmpty()) {
                estimateOpt = m_poseEstimator.estimateLowestAmbiguityPose(result);
            }

        m_latestResult2 = m_camera2.getLatestResult();
        for (var result2 : m_camera2.getAllUnreadResults()) {
            estimateOpt = m_poseEstimator2.estimateCoprocMultiTagPose(result2);
            if (estimateOpt.isEmpty()) {
                estimateOpt = m_poseEstimator2.estimateLowestAmbiguityPose(result2);
            }
        }

        m_latestResult3 = m_camera3.getLatestResult();
        for (var result3 : m_camera3.getAllUnreadResults()) {
            estimateOpt = m_poseEstimator3.estimateCoprocMultiTagPose(result3);
            if (estimateOpt.isEmpty()) {
                estimateOpt = m_poseEstimator3.estimateLowestAmbiguityPose(result3);
            }
        }

        m_latestResult4 = m_camera4.getLatestResult();
        for (var result4 : m_camera4.getAllUnreadResults()) {
            estimateOpt = m_poseEstimator4.estimateCoprocMultiTagPose(result4);
            if (estimateOpt.isEmpty()) {
                estimateOpt = m_poseEstimator4.estimateLowestAmbiguityPose(result4);
        }
    }

        estimateOpt.ifPresent(estimate -> {
            Pose2d visionPose     = estimate.estimatedPose.toPose2d();
            double captureTime  = estimate.timestampSeconds;
            int    numTagsUsed  = estimate.targetsUsed.size();

            // Basic quality check before passing to drivetrain.
            // Lesson 8 covers more sophisticated filtering.
            // For now: reject if pose is outside the field boundaries.
            if (visionPose.getX() > 0 && visionPose.getX() < 16.54 &&
                visionPose.getY() > 0 && visionPose.getY() < 8.21) {

                // Pass to drivetrain with the capture timestamp — not Timer.getFPGATimestamp()
                    
            int numTags = estimate.targetsUsed.size();

         // Average distance across all detected tags (better than using only the closest)
         double totalDist = estimate.targetsUsed.stream()
             .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
             .sum();
         double avgDist = totalDist / numTags;

         Matrix<N3, N1> stdDevs = computeVisionStdDevs(avgDist, numTags);

         // Pass to drivetrain with both pose and per-measurement stddevs
         m_drive.addVisionMeasurement(
            estimate.estimatedPose.toPose2d(),
            captureTime,
            stdDevs);

                SmartDashboard.putNumber("Vision/TagsUsed", numTagsUsed);
                SmartDashboard.putNumber("Vision/VisionX", visionPose.getX());
                SmartDashboard.putNumber("Vision/VisionY", visionPose.getY());
            }
        });
    }
    }
}