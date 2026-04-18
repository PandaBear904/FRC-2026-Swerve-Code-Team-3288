package frc.robot.subsystems;

import static frc.robot.Constants.AgitatorConstants.agitatorLeftID;
import static frc.robot.Constants.AgitatorConstants.agitatorRightID;
import static frc.robot.Constants.AgitatorConstants.agitatorVolts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase{
    private final SparkFlex agitatorLeft;
    private final SparkFlex agitatorRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    
    // Jam detection & unjam logic
    private double targetVolts = 0;
    private boolean isUnjamming = false;

    private final Timer unjamTimer  = new Timer(); // tracks how long we've been reversing
    private final Timer spinUpTimer = new Timer(); // prevents false jams on startup

    /** How long (seconds) to reverse when a jam is detected. */
    private static final double UNJAM_DURATION = 0.5;

    /**
     * How long (seconds) after setRPM() is called before jam detection activates.
     * Prevents a false-positive while the motors are still spinning up.
     */
    private static final double SPINUP_GRACE = 1.0;

    /**
     * If a motor's velocity is below this fraction of the target RPM, it is
     * considered jammed. E.g. 0.25 means less than 25% of target speed = jam.
     * Tune this on the real robot.
     */
    private static final double JAM_THRESHOLD_FRACTION = 0.25;

    public AgitatorSubsystem(){
        agitatorLeft = new SparkFlex(agitatorLeftID, MotorType.kBrushless);
        agitatorRight = new SparkFlex(agitatorRightID, MotorType.kBrushless);

        leftEncoder = agitatorLeft.getEncoder();
        rightEncoder = agitatorRight.getEncoder();

        SparkFlexConfig agitatorLConfig = new SparkFlexConfig();
        agitatorLConfig.idleMode(IdleMode.kCoast);
        agitatorLConfig.smartCurrentLimit(40);
        agitatorLConfig.inverted(true);
        agitatorLConfig.openLoopRampRate(0.12); // Ramp Rate

        SparkFlexConfig agitatorRConfig = new SparkFlexConfig();
        agitatorRConfig.idleMode(IdleMode.kCoast);
        agitatorRConfig.smartCurrentLimit(40);
        agitatorRConfig.inverted(false);
        agitatorRConfig.openLoopRampRate(0.12);

        agitatorLeft.configure(agitatorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        agitatorRight.configure(agitatorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        double leftVel  = leftEncoder.getVelocity();
        double rightVel = rightEncoder.getVelocity();

        SmartDashboard.putNumber("Agitator Left RPM",   leftVel);
        SmartDashboard.putNumber("Agitator Right RPM",  rightVel);
        SmartDashboard.putBoolean("Agitator Unjamming", isUnjamming);

        if (targetVolts != 0 && spinUpTimer.hasElapsed(SPINUP_GRACE)) {

            if (isUnjamming) {
                // Unjam duration elapsed - resume normal direction
                if (unjamTimer.hasElapsed(UNJAM_DURATION)){
                    isUnjamming = false;
                    unjamTimer.stop();
                    unjamTimer.reset();
                    setVoltage(agitatorVolts);
                }
            } else {
                // Jam check: either motor below threshold fraction of target speed
                double threshold   = Math.abs(targetVolts) * JAM_THRESHOLD_FRACTION;
                boolean leftJammed  = Math.abs(leftVel)  < threshold;
                boolean rightJammed = Math.abs(rightVel) < threshold;

                if (leftJammed || rightJammed) {
                    triggerUnjam();
                }
            }
        }
    }

        private void triggerUnjam() {
        isUnjamming = true;
        unjamTimer.reset();
        unjamTimer.start();
        setVoltage(-agitatorVolts);
    }

    public void setVoltage(double voltage){
        spinUpTimer.reset();
        spinUpTimer.start();
        if (!isUnjamming){
            agitatorLeft.setVoltage(voltage);
            agitatorRight.setVoltage(voltage);
        }
    }

    public void stopAgitator(){
        targetVolts = 0;
        isUnjamming = false;
        unjamTimer.stop();
        unjamTimer.reset();
        spinUpTimer.stop();
        spinUpTimer.reset();
        agitatorLeft.set(0);
        agitatorRight.set(0);
    }
    
}
