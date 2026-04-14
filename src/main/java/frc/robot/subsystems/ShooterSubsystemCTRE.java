package frc.robot.subsystems;

import static frc.robot.Constants.ControlConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystemCTRE extends SubsystemBase {
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityVoltage velocityReq = new VelocityVoltage(0);

    // Interpolation map: distance (meters) -> RPM
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();


    public ShooterSubsystemCTRE(){
        // Populate the RPM map from the lookup table in Constants
        for (double[] point : shooterMap) {
            rpmMap.put(point[0], point[1]);
        }
        SmartDashboard.putNumber("Shooter Target RPM", shooterTargetRPM);

        shooterLeader = new TalonFX(shooterLeaderID, "rio");
        shooterFollower = new TalonFX(shooterFollowerID, "rio");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 40;

        // Velocity Control gains
        cfg.Slot0.kP = 0.12;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.12;

        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;

        // Apply config to motors
        shooterLeader.getConfigurator().apply(cfg);
        shooterFollower.getConfigurator().apply(cfg);
        
        shooterFollower.setControl(
            new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );

    }

    public double getTargetRPM() {
        return SmartDashboard.getNumber("Shooter Target RPM", shooterTargetRPM);
    }

    /**
     * Returns the interpolated target RPM for a given distance to the target.
     * @param distanceMeters distance from the robot to the target in meters
     * @return the calculated RPM
     */
    public double getRPMFromDistance(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putBoolean("Shooter At Speed", shooterAtRPM(getTargetRPM()));
        SmartDashboard.putNumber("Shooter Distance RPM", getRPMFromDistance(
            SmartDashboard.getNumber("Shooter Test Distance (m)", 1.0)));
    }

    public void setRPM(double rpm){
        double rps = rpm / 60.0;
        shooterLeader.setControl(velocityReq.withVelocity(rps));
    }


    public double getShooterRPM() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean canRev(){
        return (getShooterRPM() <= 1000);
    }

    public boolean shooterAtRPM(double targetRPM){
        return Math.abs(getShooterRPM() - targetRPM) <= shooterRPMTolerance;
    }

    public void stopShooter(){
        shooterLeader.stopMotor();
    }


    public Command spinRPM(double rpm) {
        return this.startEnd(
            () -> setRPM(rpm),
            this::stopShooter
        ).withName("ShooterSpinRPM");
    }

    /**
     * Spins the shooter at the interpolated RPM for the given distance.
     * @param distanceMeters distance from the robot to the target in meters
     */
    public Command spinFromDistance(double distanceMeters) {
        return this.startEnd(
            () -> setRPM(getRPMFromDistance(distanceMeters)),
            this::stopShooter
        ).withName("ShooterSpinFromDistance");
    }

    /**
     * Spins the shooter using a distance supplier — re-evaluates RPM every loop cycle.
     * Use this when distance is coming from live vision data so RPM updates as the robot moves.
     * @param distanceSupplier a supplier that returns the current distance to target in meters
     */
    public Command spinFromDistanceSupplier(DoubleSupplier distanceSupplier) {
        return this.run(() -> setRPM(getRPMFromDistance(distanceSupplier.getAsDouble())))
            .finallyDo(this::stopShooter)
            .withName("ShooterSpinFromVision");
    }

    public Command spinDashboardRPM() {
        return this.startEnd(
            () -> setRPM(getTargetRPM()),
            this::stopShooter
        ).withName("ShooterSpinDashboardRPM");
    }

    public Command stopCommand() {
        return this.runOnce(this::stopShooter).withName("ShooterStop");
    }

    public Command toggleRPM(double rpm){
        return this.runOnce(() -> setRPM(rpm))
            .withName("ShooterToggleRPM");
    } 
}
