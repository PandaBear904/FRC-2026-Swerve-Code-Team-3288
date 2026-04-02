package frc.robot.subsystems;

import static frc.robot.Constants.ControlConstants.shooterRPMTolerance;
import static frc.robot.Constants.ControlConstants.shooterTargetRPM;
import static frc.robot.Constants.ShooterConstants.kickerID;
import static frc.robot.Constants.ShooterConstants.shooterFollowerID;
import static frc.robot.Constants.ShooterConstants.shooterLeaderID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShooterSubsystemCTRE extends SubsystemBase {
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;
    private final SparkFlex kicker;

    private final VelocityVoltage velocityReq = new VelocityVoltage(0);
    private final VoltageOut voltageReq = new VoltageOut(0);

    
    public ShooterSubsystemCTRE(){
        shooterLeader = new TalonFX(shooterLeaderID, "rio");
        shooterFollower = new TalonFX(shooterFollowerID, "rio");
        kicker = new SparkFlex(kickerID, MotorType.kBrushless);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

        SparkFlexConfig kickerConfig = new SparkFlexConfig();
        kickerConfig.idleMode(IdleMode.kBrake);
        kickerConfig.smartCurrentLimit(40);
        kickerConfig.inverted(false);

        kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putBoolean("Shooter At 4000", shooterAtRPM(shooterTargetRPM));
    }

    public void setRPM(double rpm){
        double rps = rpm / 60.0;
        shooterLeader.setControl(velocityReq.withVelocity(rps));
    }
    public void setVoltage(double volts){
        shooterLeader.setControl(voltageReq.withOutput(volts));
    }

    public double getShooterRPM() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean shooterAtRPM(double targetRPM){
        return Math.abs(getShooterRPM() - targetRPM) <= shooterRPMTolerance;
    }

    public void kick(double x){
        kicker.set(x);
    }

    public void stopShooter(){
        shooterLeader.stopMotor();
    }

    public void stopKicker(){
        kicker.stopMotor();
    }

    public Command setVolts(double volts){
        return this.startRun(() -> setVoltage(volts), 
            this::stopShooter);
    }

    public Command spinRPM(double rpm) {
        return this.startEnd(
            () -> setRPM(rpm),
            this::stopShooter
        ).withName("ShooterSpinRPM");
    }

    public Command stopCommand() {
        return this.runOnce(this::stopShooter).withName("ShooterStop");
    }

    public Command toggleRPM(double rpm){
        return this.runOnce(() -> setRPM(rpm))
            .withName("ShooterToggleRPM");
    } 
    public Command shootWhenReady(double rpm, double kickerPower) {
        Trigger atSpeed = new Trigger(() -> shooterAtRPM(rpm)).debounce(0.10);

        return Commands.parallel(
            // Shooter branch (requires this subsystem)
            this.run(() -> setRPM(rpm)),

            // Kicker branch (NO requirements -> avoids parallel conflict)
            Commands.sequence(
                Commands.waitUntil(atSpeed),
                Commands.runEnd(
                    () -> kick(kickerPower),
                    this::stopKicker
                )
            )
        )
        .finallyDo(interrupted -> {
            stopKicker();
            stopShooter();
        })
        .withName("ShootWhenReady");
    }
}
