// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.AgitatorConstants.agitatorVolts;
import static frc.robot.Constants.ControlConstants.intakeDownPower;
import static frc.robot.Constants.ControlConstants.intakeUpPower;
import static frc.robot.Constants.ControlConstants.reverseShooterPower;
import static frc.robot.Constants.ControlConstants.rollerPower;
import static frc.robot.Constants.OperatorConstants.driverPort;
import static frc.robot.Constants.OperatorConstants.operatorPort;
import static frc.robot.Constants.VisionConstants.desiredShotRangeMeters;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Agitator;
import frc.robot.commands.DriveIntoRange;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    public final ShooterSubsystemCTRE shooter = new ShooterSubsystemCTRE();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final AgitatorSubsystem agitator = new AgitatorSubsystem();
    public final VisionSubsystem vision = new VisionSubsystem();


    private double leftX()  { return driverController.isConnected() ? driverController.getRawAxis(0) : 0.0; } // LS X
    private double leftY()  { return driverController.isConnected() ? driverController.getRawAxis(1) : 0.0; } // LS Y
    @SuppressWarnings("unused")
    private double rightX() { return driverController.isConnected() ? driverController.getRawAxis(5) : 0.0; } // RS X
    private double rightY() { return driverController.isConnected() ? driverController.getRawAxis(2) : 0.0; } // RS Y
    // Need to have this be the joystick button
    private double speedMult() { return driverController.isConnected() && driverController.getRawButton(11) ? 0.33 : 0.66; }
    private double fullSpeed() { return driverController.isConnected() && driverController.getRawButton(12) ? 0.66: 1.0;}

    private final SwerveRequest.RobotCentric turnRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    public RobotContainer() {
        registerNamedCommands();
        configureAutoBuilder();
        autoSetUp();
        configureBindings();
    }

    public void registerNamedCommands(){
        // Set up commands for Auto
        NamedCommands.registerCommand("Intake", IntakeCommands.downAndRoller(intake, intakeDownPower, rollerPower).withTimeout(7));
        NamedCommands.registerCommand("Rev Intake", IntakeCommands.moveUpUntilLimit(intake, intakeUpPower).withTimeout(1));
        NamedCommands.registerCommand("Agitator", new Agitator(agitator, agitatorVolts).withTimeout(4));
        NamedCommands.registerCommand("Shoot", shooter.spinFromDistanceSupplier(() -> vision.getGoalDistanceMeters().orElse(desiredShotRangeMeters)).withTimeout(4));
        NamedCommands.registerCommand("TurnToTarget", new DriveIntoRange(drivetrain, vision, turnRequest, () -> brake, MaxAngularRate).withTimeout(1));
    }

     private void configureAutoBuilder(){
        try{
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds, 
                (speeds, feedforwards) -> drivetrain.setControl(
                    m_pathApplyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ), 
                new PPHolonomicDriveController(
                    new PIDConstants(3.0, 0.0, 0.0), 
                    new PIDConstants(2.5, 0.2, 0.15)
                ), 
                config, 
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
                drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
        }
    }

    private void autoSetUp(){
        // Default Do nothing
        autoChooser.setDefaultOption("Do Nothing", AutoBuilder.buildAuto("Do Nothing"));   

        autoChooser.addOption("2 Cycle Right Blue", AutoBuilder.buildAuto("Two Cycle Right Blue Side"));
        autoChooser.addOption("1 Cycle Right Blue", AutoBuilder.buildAuto("One Cycle Right Blue Side"));
        autoChooser.addOption("2 Cycle Left Blue", AutoBuilder.buildAuto("Two Cycle Left Blue Side"));
        autoChooser.addOption("1 Cycle Left Blue", AutoBuilder.buildAuto("One Cycle Left Blue Side"));
        autoChooser.addOption("Backup and Shoot Blue", AutoBuilder.buildAuto("Backup and Shoot Center Blue"));
        // autoChooser.addOption("2 Cycle Right Red", AutoBuilder.buildAuto("Two Cycle Right Red Side"));
        autoChooser.addOption("1 Cycle Right Red", AutoBuilder.buildAuto("One Cycle Right Red Side"));
        // autoChooser.addOption("2 Cycle Left Red", AutoBuilder.buildAuto("Two Cycle Left Red Side"));
        autoChooser.addOption("1 Cycle Left Red", AutoBuilder.buildAuto("One Cycle Left Red Side"));
        autoChooser.addOption("Backup and Shoot Red", AutoBuilder.buildAuto("Backup and Shoot Center Red"));
        // Send data to SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // speedMult should make a half speed 

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(()-> {

                double x = leftY();
                double y = leftX();
                double rot = rightY();

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                    x *= -1.0;
                    y *= -1.0;
                } else {
                    rot *= -1.0;
                }

                    return drive.withVelocityX(x * MaxSpeed * (speedMult() / fullSpeed()))
                                .withVelocityY(y * MaxSpeed * (speedMult() / fullSpeed()))
                                .withRotationalRate(rot * MaxAngularRate * (speedMult()) + 0.14);
            })
        );

        // Below are the driver controller buttons
        // JoystickButton squareButtonDriver = new JoystickButton(driverController, 1);
        JoystickButton xButtonDriver = new JoystickButton(driverController, 2);
        // JoystickButton oButtonDriver = new JoystickButton(driverController, 3);
        JoystickButton triangleButtonDriver = new JoystickButton(driverController, 4);
        // JoystickButton leftBumperDriver = new JoystickButton(driverController, 5);
        JoystickButton rightBumperDriver = new JoystickButton(driverController, 6);
        JoystickButton leftTriggerDriver = new JoystickButton(driverController, 7);
        JoystickButton rightTriggerDriver = new JoystickButton(driverController, 8);
        // JoystickButton shareButtonDriver = new JoystickButton(driverController, 9);
        JoystickButton optionsButtonDriver = new JoystickButton(driverController, 10);
        // leftJoystickDownDriver is used for half speed.
        // JoystickButton leftJoystickDownDriver = new JoystickButton(driverController, 11);
        // JoystickButton rightJoystickDownDriver = new JoystickButton(driverController, 12);

        // Below are the operator controller buttons
        // JoystickButton squareButtonOperator = new JoystickButton(operatorController, 1);
        // JoystickButton xButtonOperator = new JoystickButton(operatorController, 2);
        // JoystickButton oButtonOperator = new JoystickButton(operatorController, 3);
        JoystickButton triangleButtonOperator = new JoystickButton(operatorController, 4);
        JoystickButton leftBumperOperator = new JoystickButton(operatorController, 5);
        JoystickButton rightBumperOperator = new JoystickButton(operatorController, 6);
        JoystickButton leftTriggerOperator = new JoystickButton(operatorController, 7);
        JoystickButton rightTriggerOperator = new JoystickButton(operatorController, 8);
        // JoystickButton shareButtonOperator = new JoystickButton(operatorController, 9);
        // JoystickButton optionsButtonOperator = new JoystickButton(operatorController, 10);
        // JoystickButton leftJoystickDownOperator = new JoystickButton(operatorController, 11);
        // JoystickButton rightJoystickDownOperator = new JoystickButton(operatorController, 12);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        //Hey don't delete this please
        drivetrain.registerTelemetry(logger::telemeterize);

        // Turn to face best alliance goal tag
        xButtonDriver.whileTrue(new DriveIntoRange(drivetrain, vision, turnRequest, () -> brake, MaxAngularRate));

        // Put Motors in brake mode
        leftTriggerDriver.whileTrue(drivetrain.applyRequest(() -> brake));

        // X config for the wheels
        optionsButtonDriver.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-leftY(), -leftX()))
        ));

        //Intake up
        rightBumperDriver.whileTrue(IntakeCommands.moveUpUntilLimit(intake, intakeUpPower));
        //Intake down
        rightTriggerDriver.whileTrue(IntakeCommands.downAndRoller(intake, intakeDownPower, rollerPower));
        //Just run the intake
        triangleButtonDriver.whileTrue(IntakeCommands.runRollerWhileHeld(intake, rollerPower));

        // Keep AprilTag camera in detection mode for the entire auto period
        RobotModeTriggers.autonomous().whileTrue(Commands.startEnd(
            () -> vision.setAprilTagDriverMode(false),
            () -> vision.setAprilTagDriverMode(true)
        ).ignoringDisable(false));

        // Switch AprilTag camera to detection mode while either shoot button is held,
        // then back to driver mode when released
        rightTriggerOperator.or(triangleButtonOperator).or(xButtonDriver)
            .whileTrue(Commands.startEnd(
                () -> vision.setAprilTagDriverMode(false),
                () -> vision.setAprilTagDriverMode(true)
            ).ignoringDisable(false));

        // Shoot only when vision confirms aimed + in range AND within scoring window
        // rightTriggerOperator.and(vision::isReadyToShoot).and(this::isInScoringWindow)
        //     .whileTrue(shooter.spinFromDistanceSupplier(
        //         () -> vision.getGoalDistanceMeters().orElse(desiredShotRangeMeters)
        //     ));

        // Vision override — shoots regardless of aim/range check, still uses vision distance if visible
        // Falls back to desiredShotRangeMeters RPM if vision has no target
        triangleButtonOperator
            .whileTrue(shooter.spinFromDistanceSupplier(
                    () -> vision.getGoalDistanceMeters().orElse(desiredShotRangeMeters)));

        // Vision override — shoots regardless of aim/range, but still requires scoring window
        rightTriggerOperator
            .whileTrue(shooter.spinDashboardRPM()
                .alongWith(Commands.run(
                    () -> SmartDashboard.putBoolean("Vision Override Active", true)))
                .finallyDo(() -> SmartDashboard.putBoolean("Vision Override Active", false)));

        rightBumperOperator.whileTrue(new Agitator(agitator, agitatorVolts));
        leftBumperOperator.whileTrue(new Agitator(agitator, -agitatorVolts));

        leftTriggerOperator.and(shooter::canRev)
            .whileTrue(shooter.spinRPM(reverseShooterPower));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
