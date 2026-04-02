package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class ColorVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    public ColorVisionSubsystem() {
        camera = new PhotonCamera(colorCameraName);
        camera.setDriverMode(false); // Never stream to driver
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
    }

    public boolean hasTarget() {
        return latestResult.hasTargets();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (!latestResult.hasTargets()) return Optional.empty();
        return Optional.of(latestResult.getBestTarget());
    }

    // Yaw (horizontal angle) to the best detected color target, in degrees
    public Optional<Double> getTargetYawDeg() {
        return getBestTarget().map(PhotonTrackedTarget::getYaw);
    }

    // Pitch (vertical angle) to the best detected color target, in degrees
    public Optional<Double> getTargetPitchDeg() {
        return getBestTarget().map(PhotonTrackedTarget::getPitch);
    }

    // Area of the best target as a % of the image (0-100)
    public Optional<Double> getTargetArea() {
        return getBestTarget().map(PhotonTrackedTarget::getArea);
    }
}
