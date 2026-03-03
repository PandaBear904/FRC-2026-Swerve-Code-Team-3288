package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {

  /** Moves intake DOWN until the down limit switch is pressed. */
  public static Command moveDownUntilLimit(IntakeSubsystem intake, double downVolts) {
    return Commands.run(
            () -> {
              if (intake.isDownLimitPressed()) {
                intake.stopMove();
              } else {
                intake.runIntakeMove(downVolts); // downVolts should be negative or positive depending on your wiring
              }
            },
            intake
        )
        .until(intake::isDownLimitPressed)
        .finallyDo(interrupted -> intake.stopMove());
  }

  /** Moves intake UP until the up limit switch is pressed. */
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

  /** Runs the roller intake motor while the command is scheduled. */
  public static Command runRollerWhileHeld(IntakeSubsystem intake, double rollerVolts) {
    return Commands.startEnd(
        () -> intake.runIntakeOn(rollerVolts),
        intake::stopOn,
        intake
    );
  }

  /**
   * Button behavior:
   * 1) move down until limit
   * 2) then run roller until button released (because you’ll bind with whileTrue)
   */
  public static Command downThenRoller(IntakeSubsystem intake, double downVolts, double rollerVolts) {
    return moveDownUntilLimit(intake, downVolts)
        .andThen(runRollerWhileHeld(intake, rollerVolts));
  }
}