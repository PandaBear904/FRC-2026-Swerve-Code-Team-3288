package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Rotates the robot in place to face the best visible alliance goal tag.
 * Uses the alliance-aware getBestGoalTarget() from VisionSubsystem so red/blue
 * tag filtering is handled automatically. The robot holds position (vx/vy = 0)
 * and only applies a rotational rate until the yaw error is within tolerance.
 *
 * If the tag is lost mid-command (e.g. from camera shake at startup), the robot
 * holds its last rotation rather than stopping immediately. It only stops if the
 * tag has been missing for longer than TAG_LOSS_TIMEOUT_SECONDS.
 */
public class DriveIntoRange extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private final Supplier<SwerveRequest> stopRequest;
    private final SwerveRequest.RobotCentric driveRequest;

    // Will need to tune
    private final PIDController yawPID = new PIDController(4.0, 0.0, 0.25);

    private final double maxOmegaRadPerSec;

    // How long the tag must be missing before the robot stops rotating
    private static final double TAG_LOSS_TIMEOUT_SECONDS = 0.25;
    private final Timer tagLossTimer = new Timer();
    private boolean tagWasVisible = false;
    private double lastOmegaCmd = 0.0;

    public DriveIntoRange(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        SwerveRequest.RobotCentric driveRequest,
        Supplier<SwerveRequest> stopRequest,
        double maxOmegaRadPerSec
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driveRequest = driveRequest;
        this.stopRequest = stopRequest;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;

        yawPID.setTolerance(Units.degreesToRadians(2.0)); // 2 degree tolerance

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        yawPID.reset();
        tagLossTimer.stop();
        tagLossTimer.reset();
        tagWasVisible = false;
        lastOmegaCmd = 0.0;
    }

    @Override
    public void execute() {
        var yawDegOpt = vision.getGoalYawDeg();

        if (yawDegOpt.isEmpty()) {
            // Tag not visible — start or continue the loss timer
            if (tagWasVisible) {
                tagLossTimer.start();
            }
            tagWasVisible = false;

            if (tagLossTimer.get() < TAG_LOSS_TIMEOUT_SECONDS) {
                // Hold last rotation briefly while waiting for tag to reappear
                drivetrain.setControl(
                    driveRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(lastOmegaCmd)
                );
            } else {
                // Tag gone too long — stop
                drivetrain.setControl(stopRequest.get());
            }
            return;
        }

        // Tag visible — reset loss timer
        tagLossTimer.stop();
        tagLossTimer.reset();
        tagWasVisible = true;

        double yawRad = Units.degreesToRadians(yawDegOpt.get());

        // Rotate to center yaw, hold position
        lastOmegaCmd = yawPID.calculate(yawRad, 0.0);
        lastOmegaCmd = MathUtil.clamp(lastOmegaCmd, -maxOmegaRadPerSec, maxOmegaRadPerSec);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(lastOmegaCmd)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(stopRequest.get());
    }

    @Override
    public boolean isFinished() {
        return yawPID.atSetpoint();
    }
}
