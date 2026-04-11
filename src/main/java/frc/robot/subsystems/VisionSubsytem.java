package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsytem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonCamera driverCamera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    public VisionSubsytem() {
        camera = new PhotonCamera(aprilTagCameraName);

        // Driver camera — always in driver mode for low-latency streaming
        driverCamera = new PhotonCamera(driverCameraName);
        driverCamera.setDriverMode(true);

        // Start AprilTag camera in driver mode; switches to detection when aiming
        camera.setDriverMode(true);
    }

    /**
     * Switches the AprilTag camera between detection mode (for targeting)
     * and driver mode (for low-latency streaming when not aiming).
     */
    public void setAprilTagDriverMode(boolean driverMode) {
        camera.setDriverMode(driverMode);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();

        boolean aimed   = isAimedAtGoal();
        boolean inRange = isInRangeOfGoal();
        boolean ready   = aimed && inRange;

        SmartDashboard.putBoolean("Shot Ready",    ready);
        SmartDashboard.putBoolean("Aimed at Goal", aimed);
        SmartDashboard.putBoolean("In Range",      inRange);

        getBestGoalTarget().ifPresent(t -> {
            SmartDashboard.putNumber("Distance to Goal (m)", t.getBestCameraToTarget().getTranslation().getNorm());
            SmartDashboard.putNumber("Yaw to Goal (deg)",    t.getYaw());
            SmartDashboard.putNumber("Goal Tag ID",          t.getFiducialId());
        });
    }

    // Returns the tag IDs for whichever alliance we are currently on.
    // Defaults to blue if alliance is unknown.
    private int[] getAllianceTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return redTagIds;
        }
        return blueTagIds;
    }

    // Returns the best visible tag from the current alliance's goal tags.
    // "Best" = largest area (clearest / closest view).
    public Optional<PhotonTrackedTarget> getBestGoalTarget() {
        if (!latestResult.hasTargets()) return Optional.empty();

        int[] ids = getAllianceTagIds();
        return latestResult.getTargets().stream()
            .filter(t -> Arrays.stream(ids).anyMatch(id -> id == t.getFiducialId()))
            .max((a, b) -> Double.compare(a.getArea(), b.getArea()));
    }

    // Yaw (degrees) to the best visible goal tag
    public Optional<Double> getGoalYawDeg() {
        return getBestGoalTarget().map(PhotonTrackedTarget::getYaw);
    }

    // Distance (meters) to the best visible goal tag
    public Optional<Double> getGoalDistanceMeters() {
        return getBestGoalTarget().map(t -> {
            Transform3d camToTarget = t.getBestCameraToTarget();
            return camToTarget.getTranslation().getNorm();
        });
    }

    // True if yaw to the best goal tag is within tolerance
    public boolean isAimedAtGoal() {
        var yaw = getGoalYawDeg();
        return yaw.isPresent() && Math.abs(yaw.get()) <= aimToleranceDeg;
    }

    // True if distance to the best goal tag is within range + tolerance
    public boolean isInRangeOfGoal() {
        var dist = getGoalDistanceMeters();
        return dist.isPresent() && dist.get() <= (desiredShotRangeMeters + rangeToleranceM);
    }

    // True when aimed AND in range — use this to gate shooting
    public boolean isReadyToShoot() {
        return isAimedAtGoal() && isInRangeOfGoal();
    }

    // --- Generic tag lookup (still available for other uses) ---

    public boolean hasTargets() {
        return latestResult.hasTargets();
    }

    public Optional<PhotonTrackedTarget> getTag(int tagId) {
        if (!latestResult.hasTargets()) return Optional.empty();
        return latestResult.getTargets().stream()
            .filter(t -> t.getFiducialId() == tagId)
            .max((a, b) -> Double.compare(a.getArea(), b.getArea()));
    }

    public Optional<Double> getTagYawDeg(int tagId) {
        return getTag(tagId).map(PhotonTrackedTarget::getYaw);
    }

    public Optional<Double> getTagDistanceMeters(int tagId) {
        return getTag(tagId).map(t -> t.getBestCameraToTarget().getTranslation().getNorm());
    }

    public boolean isAimedAtTag(int tagId) {
        var yaw = getTagYawDeg(tagId);
        return yaw.isPresent() && Math.abs(yaw.get()) <= aimToleranceDeg;
    }

    public boolean isInRangeOfTag(int tagId, double desiredRangeMeters) {
        var dist = getTagDistanceMeters(tagId);
        return dist.isPresent() && dist.get() <= (desiredRangeMeters + rangeToleranceM);
    }
}
