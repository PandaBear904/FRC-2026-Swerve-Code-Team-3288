package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstats.intakeMoveID;
import static frc.robot.Constants.IntakeConstats.intakeOnID;
import static frc.robot.Constants.IntakeConstats.limitSwitchDown;
import static frc.robot.Constants.IntakeConstats.limitSwitchUp;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMove;
    private final SparkMax intakeOn;
    private final DigitalInput intakeLimitDown;
    private final DigitalInput intakeLimitUp;

    public IntakeSubsystem(){
        intakeMove = new SparkMax(intakeMoveID, MotorType.kBrushless);
        intakeOn = new SparkMax(intakeOnID, MotorType.kBrushless);
        intakeLimitDown = new DigitalInput(limitSwitchDown);
        intakeLimitUp = new DigitalInput(limitSwitchUp);

        SparkMaxConfig intakeMoveConfig = new SparkMaxConfig();
        intakeMoveConfig.smartCurrentLimit(40);
        intakeMoveConfig.idleMode(IdleMode.kBrake);
        intakeMoveConfig.inverted(false);

        SparkMaxConfig intakeOnConfig = new SparkMaxConfig();
        intakeOnConfig.smartCurrentLimit(40);
        intakeOnConfig.idleMode(IdleMode.kCoast);
        intakeOnConfig.inverted(false);

        intakeMove.configure(intakeMoveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeOn.configure(intakeOnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        return intakeLimitUp.get(); 
    }

    public void runIntakeMove(double volts){
        intakeMove.setVoltage(volts);
    }

    public void runIntakeOn(double volts){
        intakeOn.setVoltage(volts);
    }
    
    public void stopMove() {
        intakeMove.stopMotor();
    }

    public void stopOn() {
        intakeOn.stopMotor();
    }
}
