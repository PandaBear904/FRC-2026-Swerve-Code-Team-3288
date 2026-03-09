package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class testAuto extends SequentialCommandGroup {
    
    public testAuto(
        ShooterSubsystemCTRE shooter,
        AgitatorSubsystem agitator,
        CommandSwerveDrivetrain drivetrain) {

            // addCommands(
            // final var idle = new SwerveRequest.Idle();
            // // Reset our field centric heading to match the robot
            // // facing away from our alliance station wall (0 deg).
            // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // // Then slowly drive forward (away from us) for 5 seconds.
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-0.5)
            //         .withVelocityY(0)
            //         .withRotationalRate(0)
            // ).withTimeout(2.5),
            // // Finally idle for the rest of auton
            // drivetrain.applyRequest(() -> idle),
            //     // Set the shooter to blank RPM 
            //     Commands.runOnce(() -> shooter.setRPM(shooterTargetRPM)),
            //     // Wait until the shooter is at 4000 RPM
            //     Commands.waitUntil(() -> shooter.shooterAtRPM(shooterRPMTolerance)),
            //     //This might work??
            //     Commands.run(() -> shooter.kick(kickerPower))
            //        .alongWith(Commands.run(() -> agitator.setVoltage(-agitatorPower), agitator)).withTimeout(2.0),
            //     // Turn on the agitator on
            //     // Stop everything
            //     Commands.runOnce(agitator::stopAgitator, agitator),
            //     Commands.runOnce(shooter::stopShooter, shooter),
            //     Commands.runOnce(shooter::stopKicker, shooter)
            // );        }
}}
