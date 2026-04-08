package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
// Intake logic should work 🐼

public class IntakeCommands {

  public static Command moveDownUntilLimit(IntakeSubsystem intake, double downVolts) {
    return Commands.run(
            () -> {
              if (intake.isDownLimitPressed()) {
                intake.stopMove();
              } else {
                intake.runIntakeMove(downVolts); 
              }
            },
            intake
        )
        .until(intake::isDownLimitPressed)
        .finallyDo(interrupted -> intake.stopMove());
  }
  


  public static Command moveUpUntilLimit(IntakeSubsystem intake, double upVolts) {
    return Commands.run(
            () -> {
              if (intake.isUpLimitPressed()) {
                intake.stopMove();
              } else {
                intake.runIntakeMove(upVolts);
              }
            },
            intake
        )
        .until(intake::isUpLimitPressed)
        .finallyDo(interrupted -> intake.stopMove());
  } 

  public static Command runRollerWhileHeld(IntakeSubsystem intake, double rollerVolts) {
    return Commands.startEnd(
        () -> intake.runIntakeOn(rollerVolts),
        intake::stopOn,
        intake
    );
  }

  
  public static Command downThenRoller(IntakeSubsystem intake, double downVolts, double rollerVolts) {
    return moveDownUntilLimit(intake, downVolts)
        .alongWith(runRollerWhileHeld(intake, rollerVolts));
  }
  
}