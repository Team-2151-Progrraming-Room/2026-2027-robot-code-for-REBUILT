package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_backRightCamera = new PhotonCamera("BackRight");
    private final PhotonCamera m_backLeftCamera = new PhotonCamera("BackLeft");
    private final PhotonCamera m_frontRightCamera = new PhotonCamera("FrontRight");
    private final PhotonCamera m_frontLeftCamera = new PhotonCamera("FrontLeft");

    private PhotonPipelineResult m_latestResult = new PhotonPipelineResult();

    @Override
    public void periodic(){        
        m_latestResult = m_backRightCamera.getLatestResult();
        m_latestResult = m_backLeftCamera.getLatestResult();
        m_latestResult = m_frontRightCamera.getLatestResult();
        m_latestResult = m_frontLeftCamera.getLatestResult();

        SmartDashboard.putBoolean("Vision/HasTargets", m_latestResult.hasTargets());
        SmartDashboard.putNumber("Vision/TargetCount",
            m_latestResult.getTargets().size());
    }

    public PhotonPipelineResult getLatestResult() {
            return m_latestResult;
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (!m_latestResult.hasTargets()) {
            return Optional.empty();
        }
        
        return Optional.of(m_latestResult.getBestTarget());
    }

    PhotonTrackedTarget target = m_latestResult.getBestTarget();

   int tagId = target.getFiducialId();

   double ambiguity = target.getPoseAmbiguity();

   Transform3d camToTag = target.getBestCameraToTarget();
   double distance = camToTag.getTranslation().getNorm();

   double yaw = target.getYaw();
   double pitch = target.getPitch();
   double area = target.getArea();

   double captureTimestamp = m_latestResult.getTimestampSeconds();
}


  