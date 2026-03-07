package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;

public class testAuto extends SequentialCommandGroup {
    
    public testAuto(
        ShooterSubsystemCTRE shooter,
        AgitatorSubsystem agitator) {
            addCommands(
                // Set the shooter to 4000 RPM 
                Commands.runOnce(() -> shooter.setRPM(4000)),
                // Wait until the shooter is at 4000 RPM
                Commands.waitUntil(() -> shooter.shooterAtRPM(4000)),
                // Turn on the agitator on
                Commands.run(() -> agitator.setVoltage(6.0), agitator).withTimeout(1.0),
                // Stop everything
                Commands.runOnce(agitator::stopAgitator, agitator),
                Commands.runOnce(shooter::stopShooter, shooter),
                Commands.runOnce(shooter::stopKicker, shooter)
            );


        }
    
}
