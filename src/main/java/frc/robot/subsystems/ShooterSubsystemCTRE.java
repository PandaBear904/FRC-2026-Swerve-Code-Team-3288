package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystemCTRE extends SubsystemBase {
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityVoltage velocityReq = new VelocityVoltage(0);

    
    public ShooterSubsystemCTRE(){
        shooterLeader = new TalonFX(shooterLeaderID, "rio");
        shooterFollower = new TalonFX(shooterFollowerID, "rio");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 40;

        // Velocity Control gain (NEED TO CHANGE!!!!!!!!!!!!!)
        cfg.Slot0.kP = 0.12;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.12;

        // Apply config to motors
        shooterLeader.getConfigurator().apply(cfg);
        shooterFollower.getConfigurator().apply(cfg);
        
        shooterFollower.setControl(
            new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned)
        );
    }

    @Override
    public void periodic() {
        double shooterRPS = shooterLeader.getVelocity().getValueAsDouble();
        double shooterRPM = shooterRPS * 60.0;

        SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    }

    public void setRPM(double rpm){
        double rps = rpm / 60.0;
        shooterLeader.setControl(velocityReq.withVelocity(rps));
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

    public Command stopCommand() {
        return this.runOnce(this::stopShooter).withName("ShooterStop");
    }

    public Command toggleRPM(double rpm){
        return this.runOnce(() -> setRPM(rpm))
            .withName("ShooterToggleRPM");
    } 
}
