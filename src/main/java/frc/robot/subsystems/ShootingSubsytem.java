package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ShootingConstants.*;

public class ShootingSubsytem extends SubsystemBase {
    private final SparkMax shootingLeader;
    private final SparkMax shootingFollower1;
    //private final SparkMax shootingFollower2;

    public ShootingSubsytem(){
        shootingLeader = new SparkMax(shootingMotorID , MotorType.kBrushless);
        shootingFollower1 = new SparkMax(followingShootingMotor1ID, MotorType.kBrushless);
        //shootingFollower2 = new SparkMax(followingShootingMotor2ID, null);

        SparkMaxConfig shootingConfig = new SparkMaxConfig();
        shootingConfig.smartCurrentLimit(maxShootingV);
        shootingConfig.follow(shootingLeader);
        shootingFollower1.configure(shootingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //shootingFollower2.configure(shootingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        shootingConfig.disableFollowerMode();
        shootingLeader.configure(shootingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
    }

    public void spinUp(){
        shootingLeader.setVoltage(spinUpShootingV);
    }

    public void shoot(){
        shootingLeader.setVoltage(maxShootingV);
    }

    public void noShoot(){
        shootingLeader.set(0);
    }

    public Command startUp(){
        return this.run(() -> spinUp());
    }

    public Command FIRE(){
        return this.run(() -> shoot());
    }

    @Override
    public void periodic(){}

}
