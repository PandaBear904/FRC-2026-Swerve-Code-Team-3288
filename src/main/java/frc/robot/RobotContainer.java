// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveIntoRange;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsytem;
import frc.robot.subsystems.ShooterSubsytem;

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

    private final GenericHID driverController = new GenericHID(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsytem vision = new VisionSubsytem();
    public final ShooterSubsytem shooter = new ShooterSubsytem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    private double leftX()  { return driverController.getRawAxis(0); } // LS X
    private double leftY()  { return driverController.getRawAxis(1); } // LS Y
    private double rightX() { return driverController.getRawAxis(5); } // RS X
    //private double rightY() { return driverController.getRawAxis(5); } // RS Y

    final int SPEAKER_TAG = 7;      // change to your tag
    final double RANGE_M = 2.0;     // stop at 2 meters


    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
        .withDeadband(0.05)
        .withRotationalDeadband(0.05);


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-leftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        //need to make sure these are right
        JoystickButton squareButton = new JoystickButton(driverController, 1);
        JoystickButton xButton = new JoystickButton(driverController, 2);
        JoystickButton oButton = new JoystickButton(driverController, 3);
        JoystickButton triangleButton = new JoystickButton(driverController, 4);

        //Need to find the # for this one
        JoystickButton leftBumper = new JoystickButton(driverController, 5);
        JoystickButton rightBumper = new JoystickButton(driverController, 6);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        xButton.whileTrue(drivetrain.applyRequest(() -> brake));
        squareButton.whileTrue(drivetrain.applyRequest(() ->
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
        leftBumper.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        oButton.whileTrue(
            new DriveIntoRange(
                drivetrain, 
                vision, 
                robotDrive, 
                () -> idle, 
                SPEAKER_TAG,
                RANGE_M, 
                MaxSpeed, 
                MaxAngularRate)
        );

        triangleButton.whileTrue(new Shoot(shooter,8));

        rightBumper.whileTrue(IntakeOn.create(intake, 3, 6));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
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
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
