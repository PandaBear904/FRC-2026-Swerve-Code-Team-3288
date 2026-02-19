package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.IntakeConstats.*;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMove;
    private final SparkMax intakeOn;
    private final DigitalInput intakeMoveLimit;

    public IntakeSubsystem(){
        intakeMove = new SparkMax(intakeMoveID, MotorType.kBrushless);
        intakeOn = new SparkMax(intakeOnID, MotorType.kBrushless);
        intakeMoveLimit = new DigitalInput(limitSwitch);

        SparkMaxConfig intakeMoveConfig = new SparkMaxConfig();
        intakeMoveConfig.smartCurrentLimit(40);
        intakeMoveConfig.inverted(false);

        SparkMaxConfig intakeOnConfig = new SparkMaxConfig();
        intakeOnConfig.smartCurrentLimit(40);
        intakeOnConfig.inverted(false);

        intakeMove.configure(intakeMoveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeOn.configure(intakeOnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean intakeOut() {
        return intakeMoveLimit.get();
    }

    public void runIntakeMove(double volts){
        intakeMove.setVoltage(volts);
    }

    public void runIntakeOn(double volts){
        intakeOn.setVoltage(volts);
    }
    
    public void stopIntakeOut(){
        intakeMove.set(0);
    }

    public void stopIntake(){
        intakeOn.set(0);
    }
}
