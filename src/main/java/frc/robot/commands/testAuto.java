package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;
import static frc.robot.Constants.ControlConstants.*;

public class testAuto extends SequentialCommandGroup {
    
    public testAuto(
        ShooterSubsystemCTRE shooter,
        AgitatorSubsystem agitator) {
            addCommands(
                // Set the shooter to blank RPM 
                Commands.runOnce(() -> shooter.setRPM(shooterTargetRPM)),
                // Wait until the shooter is at 4000 RPM
                Commands.waitUntil(() -> shooter.shooterAtRPM(shooterRPMTolerance)),
                //This might work??
                Commands.run(() -> shooter.kick(kickerPower))
                    .alongWith(Commands.run(() -> agitator.setVoltage(-agitatorPower), agitator)).withTimeout(2.0),
                // Turn on the agitator on
                // Stop everything
                Commands.runOnce(agitator::stopAgitator, agitator),
                Commands.runOnce(shooter::stopShooter, shooter),
                Commands.runOnce(shooter::stopKicker, shooter)
            );


        }
    
}
