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
    public static final int kickerID = 16;

  }

  public static class AgitatorConstants {
    public static final int agitatorLeftID = 17;
    public static final int agitatorRightID = 18;
  }

  public static class VisionConstants{
    public static final String aprilTagCameraName = "AprilTag";
    public static final String colorCameraName = "Color";

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

    // TODO: Tune — stop chasing color target when its area fills this % of the image
    public static final double colorChaseStopAreaPercent = 5.0;
  }

  public static class IntakeConstats {
    public static final int intakeMoveID = 19;
    public static final int intakeOnID = 20;
    public static final int limitSwitchDown = 0;
    public static final int limitSwitchUp = 1;
 }

 public static class ControlConstants {
    public static final double shooterTargetRPM = 5400.0;
    public static final double shooterRPMTolerance = 200;
    public static final double kickerPower = 0.6;
    public static final double agitatorPower = 6.0;
    public static final double reverseShooterPower = -3000;

    public static final double intakeUpPower = 8.0;
    public static final double intakeDownPower = -6.0;
    public static final double rollerPower = 6.0;
 }

}
