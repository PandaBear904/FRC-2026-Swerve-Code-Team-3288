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

    //Theses slow down the robot
    public static final double forwardSpeed = 0.7;
    public static final double turnSpeed = 0.8;

    //Forward and Turning Axis
    public static final int forwardAxis = 0;
    public static final int turnAxis = 1;
  }

  public static class ShooterConstants {
    //Motor CAN ID
    public static final int shooterLeaderID = 14;
    public static final int shooterFollowerID = 15;
  }

  public static class VisionConstants{
    public static final String cameraName = "USB_2.0_Camera";

    //Range from pitch (all in meters)
    public static final double cameraHeightMeters = 0.35;
    public static final double targetHeightMeters = 0.45;
    public static final double cameraPitchRadians = Math.toRadians(20.0);

    public static final double aimToleranceDeg = 1.5;
    public static final double rangeToleranceM = 0.10;
  }

  public static class IntakeConstats {
    public static final int intakeMoveID = 16;
    public static final int intakeOnID = 17;
    public static final int limitSwitch = 0;
  }

}
