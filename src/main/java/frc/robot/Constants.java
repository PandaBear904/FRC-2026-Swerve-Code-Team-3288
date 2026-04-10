// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static class OperatorConstants {
    //Controller Ports
    //Can change on the Driver Station
    public static final int driverPort = 0; 
    public static final int operatorPort = 1;
 }

  public static class ShooterConstants {
    //Motor CAN ID
    public static final int shooterLeaderID = 14;
    public static final int shooterFollowerID = 15;

    // Distance-to-RPM lookup table
    // Each row is { distanceMeters, targetRPM }
    // TODO: Add more rows between min and max for better accuracy

    public static final double[][] shooterMap = {
      { 1.2446,  3750 },  // min distance (meters) and min RPM
      { 3.5052,  5400 },  // max distance (meters) and max RPM
    };
  }

  public static class AgitatorConstants {
    public static final int agitatorLeftID = 17;
    public static final int agitatorRightID = 18;

    // Target RPM for agitator — TODO: tune this for your robot
    public static final double agitatorTargetRPM = 3000.0;

    // Closed-loop PID + feedforward gains for SparkFlex velocity control
    // kFF = 1 / Vortex free speed RPM (~6784) — tune kP if RPM is not tracking well
    public static final double agitatorKP  = 0.0001;
    public static final double agitatorKI  = 0.0;
    public static final double agitatorKD  = 0.0;
    public static final double agitatorKFF = 0.000148;
  }

  public static class VisionConstants{
    public static final String aprilTagCameraName = "AprilTag";

    //Range from pitch (all in meters)
    public static final double cameraHeightMeters = 0.35;
    public static final double targetHeightMeters = 0.45;
    public static final double cameraPitchRadians = Math.toRadians(20.0);

    public static final double aimToleranceDeg = 1.5;
    public static final double rangeToleranceM = 0.10;

    public static final int[] redTagIds  = { 2, 3, 4, 5, 8, 11};
    public static final int[] blueTagIds = { 18, 21, 24, 25, 26, 27};

    // TODO: Tune this to the max distance you can reliably score from
    public static final double desiredShotRangeMeters = 2.0;

  }

  public static class IntakeConstats {
    public static final int intakeMoveID = 19;
    public static final int intakeOnID = 20;
    public static final int limitSwitchDown = 0;
    public static final int limitSwitchUp = 1;
 }

 public static class ControlConstants {
    public static final double shooterTargetRPM = 3750.0;
    public static final double shooterRPMTolerance = 200;
    public static final double reverseShooterPower = -3000;

    public static final double intakeUpPower = 8.0;
    public static final double intakeDownPower = -6.0;
    public static final double rollerPower = 3500; //This is in RPM I too lazy to change the name
 }

}
