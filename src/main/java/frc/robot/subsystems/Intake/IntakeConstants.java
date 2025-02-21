// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
  public static final int smallRollerId = 9;
  public static final int lowRollerId = 10;
  public static final int supRollerId = 11;

  public static final double motorReduction = 60;

  public static final int currentLimit = 60;

  public static final Boolean smallRollerInverted = false;
  public static final Boolean lowRollerInverted = false;
  public static final Boolean supRollerInverted = false;

  public static class WristConstants {
    public static final int wristMotorId = 8;

    public static final double motorReduction = 60;

    public static final int currentLimit = 60;

    public static final double realKp = 0.0;
    public static final double realKd = 0.0;
    public static final double realKs = 0.0;
    public static final double realKv = 0.1;

    public static final double simKp = 0.10;
    public static final double simKd = 0.0;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;

    public static final Boolean wristMotorInverted = false;
  }
}
