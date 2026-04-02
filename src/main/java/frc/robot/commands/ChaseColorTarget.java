package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ColorVisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static frc.robot.Constants.VisionConstants.*;

public class ChaseColorTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final ColorVisionSubsystem colorVision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // Rotates the robot to keep the target centered horizontally
    private final PIDController yawPID   = new PIDController(4.0, 0.0, 0.1);
    // Drives forward/back based on how much of the image the target fills
    private final PIDController areaPID  = new PIDController(0.05, 0.0, 0.0);

    private final double maxSpeedMps;
    private final double maxOmegaRadPerSec;

    public ChaseColorTarget(
        CommandSwerveDrivetrain drivetrain,
        ColorVisionSubsystem colorVision,
        double maxSpeedMps,
        double maxOmegaRadPerSec
    ) {
        this.drivetrain        = drivetrain;
        this.colorVision       = colorVision;
        this.maxSpeedMps       = maxSpeedMps;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;

        yawPID.setTolerance(2.0);           // degrees
        areaPID.setTolerance(0.5);          // % of image

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        yawPID.reset();
        areaPID.reset();
    }

    @Override
    public void execute() {
        var yawOpt  = colorVision.getTargetYawDeg();
        var areaOpt = colorVision.getTargetArea();

        // No target visible — stop and wait
        if (yawOpt.isEmpty() || areaOpt.isEmpty()) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }

        double yawRad = Units.degreesToRadians(yawOpt.get());
        double area   = areaOpt.get();

        // Positive vx = drive forward (area too small = too far away)
        double vxCmd    = areaPID.calculate(area, colorChaseStopAreaPercent);
        double omegaCmd = yawPID.calculate(yawRad, 0.0);

        vxCmd    = MathUtil.clamp(vxCmd,    -maxSpeedMps * 0.5, maxSpeedMps * 0.5);
        omegaCmd = MathUtil.clamp(omegaCmd, -maxOmegaRadPerSec, maxOmegaRadPerSec);

        drivetrain.setControl(driveRequest
            .withVelocityX(vxCmd)
            .withVelocityY(0.0)
            .withRotationalRate(omegaCmd));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return areaPID.atSetpoint() && yawPID.atSetpoint();
    }
}
