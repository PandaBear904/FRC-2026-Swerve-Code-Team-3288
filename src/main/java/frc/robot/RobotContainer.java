// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ControlConstants.agitatorPower;
import static frc.robot.Constants.ControlConstants.intakeDownPower;
import static frc.robot.Constants.ControlConstants.intakeUpPower;
import static frc.robot.Constants.ControlConstants.reverseShooterPower;
import static frc.robot.Constants.ControlConstants.rollerPower;
import static frc.robot.Constants.OperatorConstants.driverPort;
import static frc.robot.Constants.OperatorConstants.operatorPort;

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
import frc.robot.commands.ChaseColorTarget;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ColorVisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystemCTRE;
import frc.robot.subsystems.VisionSubsytem;

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
    public final ShooterSubsystemCTRE shooter = new ShooterSubsystemCTRE();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final AgitatorSubsystem agitator = new AgitatorSubsystem();
    public final VisionSubsytem vision = new VisionSubsytem();
    public final ColorVisionSubsystem colorVision = new ColorVisionSubsystem();

    private double leftX()  { return driverController.getRawAxis(0); } // LS X
    private double leftY()  { return driverController.getRawAxis(1); } // LS Y
    @SuppressWarnings("unused")
    private double rightX() { return driverController.getRawAxis(5); } // RS X
    private double rightY() { return driverController.getRawAxis(2); } // RS Y
    // Need to have this be the joystick button
    private double speedMult() { return driverController.getRawButton(11) ? 0.25 : 0.75; }

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    public RobotContainer() {
        registerNamedCommands();
        configureAutoBuilder();

        // Set up AutoChooser
        autoChooser.setDefaultOption("Do Nothing", AutoBuilder.buildAuto("Do Nothing"));
        autoChooser.addOption("Intake Test", AutoBuilder.buildAuto("Intake Only Auto"));
        autoChooser.addOption("Test Auto", AutoBuilder.buildAuto("Test Auto"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    public void registerNamedCommands(){
        // Set up commands for Auto
        NamedCommands.registerCommand("Intake", IntakeCommands.downThenRoller(intake, intakeDownPower, rollerPower).withTimeout(3));
        NamedCommands.registerCommand("Rev Intake", IntakeCommands.moveUpUntilLimit(intake, intakeUpPower).withTimeout(1.5));
        NamedCommands.registerCommand("Agitator", new Agitator(agitator, agitatorPower));
        NamedCommands.registerCommand("Shoot", shooter.spinDashboardRPM());

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
                    if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue){
                        x *= -1.0;
                        y *= -1.0;
                    } else {
                        x *= 1.0;
                        y *= 1.0;
                    }

                    return drive.withVelocityX(x * MaxSpeed * speedMult())
                                .withVelocityY(y * MaxSpeed * speedMult())
                                .withRotationalRate(rot * MaxAngularRate * speedMult());
            })
        );

        // Below are the driver controller buttons
        // JoystickButton squareButtonDriver = new JoystickButton(driverController, 1);
        // JoystickButton xButtonDriver = new JoystickButton(driverController, 2);
        // JoystickButton oButtonDriver = new JoystickButton(driverController, 3);
        JoystickButton triangleButtonDriver = new JoystickButton(driverController, 4);
        JoystickButton leftBumperDriver = new JoystickButton(driverController, 5);
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

        drivetrain.registerTelemetry(logger::telemeterize);

        // Put Motors in brake mode
        leftTriggerDriver.whileTrue(drivetrain.applyRequest(() -> brake));

        // Chase the color target while held; stops when close enough or button released
        leftBumperDriver.whileTrue(new ChaseColorTarget(drivetrain, colorVision, MaxSpeed, MaxAngularRate));
        // X config for the wheels
        optionsButtonDriver.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-leftY(), -leftX()))
        ));

        //Intake up
        rightBumperDriver.whileTrue(IntakeCommands.moveUpUntilLimit(intake, intakeUpPower));
        //Intake down
        rightTriggerDriver.whileTrue(IntakeCommands.downThenRoller(intake, intakeDownPower, rollerPower));
        //Just run the intake
        triangleButtonDriver.whileTrue(IntakeCommands.runRollerWhileHeld(intake, rollerPower));
        
        // Shoot only when vision confirms aimed + in range
        rightTriggerOperator.and(vision::isReadyToShoot)
            .whileTrue(shooter.spinDashboardRPM());

        // Vision override — shoots regardless, flags dashboard so driver knows
        triangleButtonOperator
            .whileTrue(shooter.spinDashboardRPM()
                .alongWith(Commands.run(
                    () -> SmartDashboard.putBoolean("Vision Override Active", true)))
                .finallyDo(() -> SmartDashboard.putBoolean("Vision Override Active", false)));

        rightBumperOperator.whileTrue(new Agitator(agitator, agitatorPower));
        leftBumperOperator.whileTrue(new Agitator(agitator, -agitatorPower));
        leftTriggerOperator.whileTrue(shooter.spinRPM(reverseShooterPower));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
