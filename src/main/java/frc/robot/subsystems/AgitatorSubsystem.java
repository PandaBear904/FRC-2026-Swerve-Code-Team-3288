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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase {

    private final SparkFlex agitatorLeft;
    private final SparkFlex agitatorRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    // Jam detection & unjam logic
    private double targetRPM = 0;
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
        double leftVel  = leftEncoder.getVelocity();
        double rightVel = rightEncoder.getVelocity();

        SmartDashboard.putNumber("Agitator Left RPM",   leftVel);
        SmartDashboard.putNumber("Agitator Right RPM",  rightVel);
        SmartDashboard.putBoolean("Agitator Unjamming", isUnjamming);

        // Only run jam detection when motors are supposed to be spinning
        // and the spin-up grace period has passed
        if (targetRPM != 0 && spinUpTimer.hasElapsed(SPINUP_GRACE)) {

            if (isUnjamming) {
                // Unjam duration elapsed — resume normal direction
                if (unjamTimer.hasElapsed(UNJAM_DURATION)) {
                    isUnjamming = false;
                    unjamTimer.stop();
                    unjamTimer.reset();
                    leftController.setReference(targetRPM,  ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                    rightController.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                }
                // While unjamming, the reversed reference set in triggerUnjam() stays active

            } else {
                // Jam check: either motor below threshold fraction of target speed
                double threshold   = Math.abs(targetRPM) * JAM_THRESHOLD_FRACTION;
                boolean leftJammed  = Math.abs(leftVel)  < threshold;
                boolean rightJammed = Math.abs(rightVel) < threshold;

                if (leftJammed || rightJammed) {
                    triggerUnjam();
                }
            }
        }
    }

    /** Reverses both motors for UNJAM_DURATION seconds, then periodic() restores normal direction. */
    private void triggerUnjam() {
        isUnjamming = true;
        unjamTimer.reset();
        unjamTimer.start();
        leftController.setReference(-targetRPM,  ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rightController.setReference(-targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        // Reset the spin-up grace timer so jam detection doesn't fire immediately
        spinUpTimer.reset();
        spinUpTimer.start();

        if (!isUnjamming) {
            leftController.setReference(rpm,  ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            rightController.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    public void stopAgitator() {
        targetRPM = 0;
        isUnjamming = false;
        unjamTimer.stop();
        unjamTimer.reset();
        spinUpTimer.stop();
        spinUpTimer.reset();
        agitatorLeft.set(0);
        agitatorRight.set(0);
    }
}
