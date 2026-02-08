package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsytem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    public VisionSubsytem(){
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
    }

    //Check and see if there is a tag
    public boolean hasTargets(){
        return latestResult.hasTargets();
    }

    //Get tag number
    public Optional<PhotonTrackedTarget> getTag(int tagId) {
    if (!latestResult.hasTargets()) return Optional.empty();

    return latestResult.getTargets().stream()
        .filter(t -> t.getFiducialId() == tagId)
        .max((a, b) -> Double.compare(a.getArea(), b.getArea())); // choose biggest
}


    //Yaw to tag in degrees
    public Optional<Double> getTagYawDeg(int tagId) {
        return getTag(tagId).map(PhotonTrackedTarget::getYaw);
    }

    //Estimate the distance to the tag in meters
    public Optional<Double> getTagDistanceMeters(int tagId) {
        return getTag(tagId).map(t -> {
            Transform3d camToTarget = t.getBestCameraToTarget();
            return camToTarget.getTranslation().getNorm();
        });
    }

    //Is aimed at tag
    public boolean isAimedAtTag(int tagId){
        var yaw = getTagYawDeg(tagId);
        return yaw.isPresent() && Math.abs(yaw.get()) <= aimToleranceDeg;
    }

    //Is in shooting range
    public boolean isInRangeOfTag(int tagId, double desiredRangeMeters) {
        var dist = getTagDistanceMeters(tagId);
        return dist.isPresent() && dist.get() <=(desiredRangeMeters + rangeToleranceM);
    }
}
