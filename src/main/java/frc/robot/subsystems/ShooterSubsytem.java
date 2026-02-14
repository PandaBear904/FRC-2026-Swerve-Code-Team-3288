package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsytem extends SubsystemBase {
    private final SparkMax shooterMotor;
    private final RelativeEncoder shooterEncoder;

    public ShooterSubsytem(){
        shooterMotor = new SparkMax(shooterID, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();

        SparkMaxConfig shootConfig = new SparkMaxConfig();
        shootConfig.idleMode(IdleMode.kCoast);
        shootConfig.smartCurrentLimit(40);
        shootConfig.inverted(true);
        shootConfig.openLoopRampRate(1); //Smooth spin-up, helps brownouts

        shooterMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        double shooterRPM = shooterEncoder.getVelocity();

        SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    }

    //Set the voltage for the shooter
    public void setVoltage(double voltage){
        shooterMotor.setVoltage(voltage);
    }

    public void stopShooter(){
        shooterMotor.set(0);
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

    public Command toggleVolts(double volts) {
        return this.runOnce(() -> {
            setVoltage(volts);
        }).withName("ShooterToggleVolts");
    }
}