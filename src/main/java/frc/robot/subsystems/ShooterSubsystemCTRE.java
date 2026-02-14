package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystemCTRE extends SubsystemBase {
    private final TalonFX shooter;

    private final VoltageOut voltageReq = new VoltageOut(0);
    
    public ShooterSubsystemCTRE(){
        shooter = new TalonFX(shooterID, "rio");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotorOutputConfigs out = new MotorOutputConfigs();
        out.NeutralMode = NeutralModeValue.Coast;

        out.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput = out;

        CurrentLimitsConfigs current = new CurrentLimitsConfigs();
        current.SupplyCurrentLimitEnable = true;
        current.SupplyCurrentLimit = 40;
        cfg.CurrentLimits = current;

        shooter.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        double shooterRPS = shooter.getVelocity().getValueAsDouble();
        double shooterRPM = shooterRPS * 60.0;

        SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    }

    public void setVoltage(double volts){
        shooter.setControl(voltageReq.withOutput(volts));
    }

    public void stopShooter(){
        shooter.setControl(voltageReq.withOutput(0.0));
    }

    public Command spinVolts(double volts) {
        return this.startEnd(
            () -> setVoltage(volts),
            this::stopShooter
        ).withName("ShooterSpinVolts");
    }

    public Command stopCommand() {
        return this.runOnce(this::stopShooter).withName("ShooterStop");
    }

    public Command toggleVolts(double volts){
        return this.runOnce(() -> setVoltage(volts))
            .withName("ShooterToggleVolts");
    } 
}
