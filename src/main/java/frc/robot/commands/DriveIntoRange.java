package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Rotates the robot in place to face the best visible alliance goal tag.
 * Uses the alliance-aware getBestGoalTarget() from VisionSubsystem so red/blue
 * tag filtering is handled automatically. The robot holds position (vx/vy = 0)
 * and only applies a rotational rate until the yaw error is within tolerance.
 */
public class DriveIntoRange extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private final Supplier<SwerveRequest> stopRequest;
    private final SwerveRequest.RobotCentric driveRequest;

    // Will need to tune
    private final PIDController yawPID = new PIDController(4.0, 0.0, 0.25);

    private final double maxOmegaRadPerSec;

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
    }

    @Override
    public void execute() {
        // Use alliance-aware best goal tag — no tag visible means hold still
        var yawDegOpt = vision.getGoalYawDeg();

        if (yawDegOpt.isEmpty()) {
            drivetrain.setControl(stopRequest.get());
            return;
        }

        double yawRad = Units.degreesToRadians(yawDegOpt.get());

        // Rotate to center yaw, hold position
        double omegaCmd = yawPID.calculate(yawRad, 0.0);
        omegaCmd = MathUtil.clamp(omegaCmd, -maxOmegaRadPerSec, maxOmegaRadPerSec);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omegaCmd)
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
