package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.AgitatorConstants.*;

public class AgitatorSubsystem extends SubsystemBase{
    private final SparkFlex agitatorLeft;
    private final SparkFlex agitatorRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public AgitatorSubsystem(){
        agitatorLeft = new SparkFlex(agitatorLeftID, MotorType.kBrushless);
        agitatorRight = new SparkFlex(agitatorRightID, MotorType.kBrushless);

        leftEncoder = agitatorLeft.getEncoder();
        rightEncoder = agitatorRight.getEncoder();

        SparkFlexConfig agitatorLConfig = new SparkFlexConfig();
        agitatorLConfig.idleMode(IdleMode.kCoast);
        agitatorLConfig.smartCurrentLimit(40);
        agitatorLConfig.inverted(false);
        agitatorLConfig.openLoopRampRate(0.12); // Ramp Rate

        SparkFlexConfig agitatorRConfig = new SparkFlexConfig();
        agitatorRConfig.apply(agitatorLConfig);
        // agitatorRConfig.inverted(false);
        agitatorRConfig.follow(agitatorLeft);

        agitatorLeft.configure(agitatorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        agitatorRight.configure(agitatorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        // Get the encoder stuff
        double leftPos = leftEncoder.getPosition();
        double rightPos = rightEncoder.getPosition();

        SmartDashboard.putNumber("Left Position: ", leftPos);
        SmartDashboard.putNumber("Right Position: ", rightPos);
    }

    public void setVoltage(double voltage){
        agitatorLeft.setVoltage(voltage);
    }

    public void stopAgitator(){
        agitatorLeft.set(0);
    }
    
}
