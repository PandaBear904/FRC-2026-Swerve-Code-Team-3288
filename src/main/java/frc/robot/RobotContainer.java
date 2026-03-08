// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.commands.Agitator;
import frc.robot.commands.DriveIntoRange;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.testAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;
import frc.robot.subsystems.VisionSubsytem;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.ControlConstants.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final GenericHID driverController = new GenericHID(driverPort);
    private final GenericHID operatorController = new GenericHID(operatorPort);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //public final VisionSubsytem vision = new VisionSubsytem();
    public final ShooterSubsystemCTRE shooter = new ShooterSubsystemCTRE();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final AgitatorSubsystem agitator = new AgitatorSubsystem();

    private double leftX()  { return driverController.getRawAxis(0); } // LS X
    private double leftY()  { return driverController.getRawAxis(1); } // LS Y
    private double rightX() { return driverController.getRawAxis(5); } // RS X
    private double rightY() { return driverController.getRawAxis(2); } // RS Y
    // Need to have this be the joystick button
    private double speedMult() { return driverController.getRawButton(11) ? 0.25 : 0.75; }

    // Vision stuff and things 🐼
    final int HUB_TAG = 7;
    // stop at 2 meters
    final double RANGE_M = 2.0;

    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
        .withDeadband(0.05)
        .withRotationalDeadband(0.05);


    public RobotContainer() {
        configureBindings();
        configureAuto();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // speedMult should make a half speed 🐼

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(()-> {
                double x = -leftY();
                double y = -leftX();
                double rot = -rightY();

                var alliance = DriverStation.getAlliance();
                    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
                        x *= -1.0;
                        y *= 1.0;
                    }

                    return drive.withVelocityX(x * MaxSpeed * speedMult())
                                .withVelocityY(y * MaxSpeed * speedMult())
                                .withRotationalRate(rot * MaxAngularRate * speedMult());
            })
        );

        /* drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-leftY() * MaxSpeed * speedMult()) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftX() * MaxSpeed * speedMult()) // Drive left with negative X (left)
                    .withRotationalRate(-rightY() * MaxAngularRate * speedMult()) // Drive counterclockwise with negative X (left)
            )
        );*/

        // Below are the driver controller buttons
        JoystickButton squareButtonDriver = new JoystickButton(driverController, 1);
        JoystickButton xButtonDriver = new JoystickButton(driverController, 2);
        JoystickButton oButtonDriver = new JoystickButton(driverController, 3);
        JoystickButton triangleButtonDriver = new JoystickButton(driverController, 4);
        JoystickButton leftBumperDriver = new JoystickButton(driverController, 5);
        JoystickButton rightBumperDriver = new JoystickButton(driverController, 6);
        JoystickButton leftTriggerDriver = new JoystickButton(driverController, 7);
        JoystickButton rightTriggerDriver = new JoystickButton(driverController, 8);
        JoystickButton shareButtonDriver = new JoystickButton(driverController, 9);
        JoystickButton optionsButtonDriver = new JoystickButton(driverController, 10);
        // leftJoystickDownDriver is used for half speed.
        JoystickButton leftJoystickDownDriver = new JoystickButton(driverController, 11);
        JoystickButton rightJoystickDownDriver = new JoystickButton(driverController, 12);

        // Below are the operator controller buttons
        JoystickButton squareButtonOperator = new JoystickButton(operatorController, 1);
        JoystickButton xButtonOperator = new JoystickButton(operatorController, 2);
        JoystickButton oButtonOperator = new JoystickButton(operatorController, 3);
        JoystickButton triangleButtonOperator = new JoystickButton(operatorController, 4);
        JoystickButton leftBumperOperator = new JoystickButton(operatorController, 5);
        JoystickButton rightBumperOperator = new JoystickButton(operatorController, 6);
        JoystickButton leftTriggerOperator = new JoystickButton(operatorController, 7);
        JoystickButton rightTriggerOperator = new JoystickButton(operatorController, 8);
        JoystickButton shareButtonOperator = new JoystickButton(operatorController, 9);
        JoystickButton optionsButtonOperator = new JoystickButton(operatorController, 10);
        JoystickButton leftJoystickDownOperator = new JoystickButton(operatorController, 11);
        JoystickButton rightJoystickDownOperator = new JoystickButton(operatorController, 12);



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        leftTriggerDriver.whileTrue(drivetrain.applyRequest(() -> brake));
        optionsButtonDriver.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-leftY(), -leftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // Reset the field-centric heading on left bumper press.
        leftBumperDriver.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Intake up
        rightBumperDriver.whileTrue(IntakeCommands.moveUpUntilLimit(intake, intakeUpPower));
        //Intake down
        rightTriggerDriver.whileTrue(IntakeCommands.downThenRoller(intake, intakeDownPower, rollerPower));
        triangleButtonDriver.whileTrue(IntakeCommands.runRollerWhileHeld(intake, rollerPower));

        rightTriggerOperator.whileTrue(shooter.shootWhenReady(shooterTargetRPM, kickerPower));
        rightBumperOperator.whileTrue(new Agitator(agitator, -agitatorPower));
        leftBumperOperator.whileTrue(new Agitator(agitator, agitatorPower));
        leftTriggerOperator.whileTrue(shooter.shootWhenReady(reverseShooterPower, (-kickerPower + 0.3)));

        triangleButtonOperator.whileTrue(shooter.shootWhenReady(shooterTargetRPM, kickerPower));


    }

    private void configureAuto(){
        autoChooser.setDefaultOption("Do Nothing", null);
        autoChooser.addOption("Test Auto", new testAuto(shooter, agitator));
        autoChooser.addOption("Move Auto (don't work 🐼)", null);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //Ned to move this to the commands folder 🐼
        // Simple drive forward auton
       /* final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(2.5),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );*/
    }
}
