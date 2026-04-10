package frc.robot.subsystems;
import static frc.robot.Constants.AgitatorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase {

    private final SparkFlex agitatorLeft;
    private final SparkFlex agitatorRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    public AgitatorSubsystem() {
        agitatorLeft  = new SparkFlex(agitatorLeftID,  MotorType.kBrushless);
        agitatorRight = new SparkFlex(agitatorRightID, MotorType.kBrushless);

        leftEncoder  = agitatorLeft.getEncoder();
        rightEncoder = agitatorRight.getEncoder();

        SparkFlexConfig agitatorLConfig = new SparkFlexConfig();
        agitatorLConfig.idleMode(IdleMode.kCoast);
        agitatorLConfig.smartCurrentLimit(40);
        agitatorLConfig.inverted(true);
        agitatorLConfig.closedLoopRampRate(0.12);
        agitatorLConfig.closedLoop
            .p(agitatorKP)
            .i(agitatorKI)
            .d(agitatorKD)
            .velocityFF(agitatorKFF);

        SparkFlexConfig agitatorRConfig = new SparkFlexConfig();
        agitatorRConfig.idleMode(IdleMode.kCoast);
        agitatorRConfig.smartCurrentLimit(40);
        agitatorRConfig.inverted(false);
        agitatorRConfig.closedLoopRampRate(0.12);
        agitatorRConfig.closedLoop
            .p(agitatorKP)
            .i(agitatorKI)
            .d(agitatorKD)
            .velocityFF(agitatorKFF);

        agitatorLeft.configure(agitatorLConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        agitatorRight.configure(agitatorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftController  = agitatorLeft.getClosedLoopController();
        rightController = agitatorRight.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Agitator Left RPM",  leftEncoder.getVelocity());
        SmartDashboard.putNumber("Agitator Right RPM", rightEncoder.getVelocity());
    }

    public void setRPM(double rpm) {
        leftController.setReference(rpm,  ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rightController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stopAgitator() {
        agitatorLeft.set(0);
        agitatorRight.set(0);
    }
}