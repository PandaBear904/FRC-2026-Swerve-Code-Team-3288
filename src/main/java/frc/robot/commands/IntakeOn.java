package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOn {
    public static Command create(IntakeSubsystem sub, double outSpeed, double onSpeed){
        return Commands.sequence(
            Commands.run(() -> sub.runIntakeMove(outSpeed), sub)
                .until(sub::intakeOut),

            Commands.runOnce(sub::stopIntakeOut),

            Commands.run(() -> sub.runIntakeOn(onSpeed), sub)
        );
    }
    
}
