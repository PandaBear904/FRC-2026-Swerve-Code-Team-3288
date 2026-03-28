package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstats.intakeMoveID;
import static frc.robot.Constants.IntakeConstats.intakeOnID;
import static frc.robot.Constants.IntakeConstats.limitSwitchDown;
import static frc.robot.Constants.IntakeConstats.limitSwitchUp;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMove;
    private final TalonFX intakeOn;
    private final DigitalInput intakeLimitDown;
    private final DigitalInput intakeLimitUp;

    private final VoltageOut voltageReq = new VoltageOut(0);

    public IntakeSubsystem(){
        intakeMove = new SparkMax(intakeMoveID, MotorType.kBrushless);
        intakeOn = new TalonFX(intakeOnID, "rio");
        
        intakeLimitDown = new DigitalInput(limitSwitchDown);
        intakeLimitUp = new DigitalInput(limitSwitchUp);

        SparkMaxConfig intakeMoveConfig = new SparkMaxConfig();
        intakeMoveConfig.smartCurrentLimit(40);
        intakeMoveConfig.idleMode(IdleMode.kCoast);
        intakeMoveConfig.inverted(false);
        intakeMove.configure(intakeMoveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 40;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeOn.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Limit Down", isDownLimitPressed());
        SmartDashboard.putBoolean("Limit Up", isUpLimitPressed());
    }

    // true when the DOWN limit switch is physically pressed
    public boolean isDownLimitPressed() {
        return intakeLimitDown.get();
    }

    // true when the UP limit switch is physically pressed
    public boolean isUpLimitPressed() {
        return !intakeLimitUp.get(); 
    }

    public void runIntakeMove(double volts){
        intakeMove.setVoltage(volts);
    }

    public void runIntakeOn(double volts){
        intakeOn.setControl(voltageReq.withOutput(volts));
    }
    
    public void stopMove() {
        intakeMove.stopMotor();
    }

    public void stopOn() {
        intakeOn.stopMotor();
    }
}
