package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsytem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveIntoRange extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsytem vision;

    private final Supplier<SwerveRequest> stopRequest;
    private final SwerveRequest.RobotCentric driveRequest;

    private final int tagId;
    private final double desiredRangeM;

    //Will need to mess with theses
    private final PIDController rangePID = new PIDController(1.2, 0.0, 0.0);
    private final PIDController yawPID = new PIDController(4.0, 0.0, 0.25);

    private final double maxSpeedMps;
    private final double maxOmegaRadPerSec;

    public DriveIntoRange(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsytem vision,
        SwerveRequest.RobotCentric driveRequest,
        Supplier<SwerveRequest> stopRequest,
        int tagId,
        double desiredRangeM,
        double maxSpeedMps,
        double maxOmegaRadPerSec
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driveRequest = driveRequest;
        this.stopRequest = stopRequest;
        this.tagId = tagId;
        this.desiredRangeM = desiredRangeM;
        this.maxSpeedMps = maxSpeedMps;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;

        rangePID.setTolerance(0.10); // 10cm
        yawPID.setTolerance(2.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rangePID.reset();
        yawPID.reset();
    }

    @Override
    public void execute() {
        //If no tag nothing happens
        if (!vision.hasTargets() || vision.getTag(tagId).isEmpty()){
            drivetrain.setControl(stopRequest.get());
            return;
        }

        var yawDegOpt = vision.getTagYawDeg(tagId);
        var distOpt = vision.getTagDistanceMeters(tagId);

        if (yawDegOpt.isEmpty() || distOpt.isEmpty()){
            drivetrain.setControl(stopRequest.get());
            return;
        }

        double yawRad = Units.degreesToRadians(yawDegOpt.get());
        double distM = distOpt.get();

        //forward/back speeds
        double vxCmd = rangePID.calculate(distM, desiredRangeM);
        
        //Rotate to center yaw
        double omegaCmd = yawPID.calculate(yawRad, 0.0);

        //clamp
        vxCmd = MathUtil.clamp(vxCmd, -maxSpeedMps * 0.6, maxSpeedMps * 0.6);
        omegaCmd = MathUtil.clamp(omegaCmd, -maxOmegaRadPerSec, maxOmegaRadPerSec);

        //Drive :YIPPIEEE:
        drivetrain.setControl(
            driveRequest
                .withVelocityX(vxCmd)
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
        return rangePID.atSetpoint() && yawPID.atSetpoint();
    }
}
