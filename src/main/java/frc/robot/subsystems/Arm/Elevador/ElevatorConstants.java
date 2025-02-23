// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Elevador;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int elevatorMotorId = 7;

  // Motor configuration
  public static final double elevatorMotorMaxSpeed = 4.0;

  public static final int currentLimit = 60;

  public static final boolean isInverted = false;

  // Elevator configuration
  public static final double motorReduction = 27.0;
  public static final double carriageMass = 3.0;
  public static final double drumRadius = 0.944;
  public static final double minHeight = 0.75;
  public static final double maxHeight = 1.10;
  public static final double starningHeight = 0.75;
  public static final double distancePerPulse = 2 * Math.PI / 4096.0;

  // Velocity PID configuration
  public static final double realKp = 0.0;
  public static final double realKd = 0.0;
  public static final double realKs = 0.01;
  public static final double realKv = 0.1;
  public static final double realKa = 0.0;
  public static final double realKg = 0.0;

  public static final double simKp = 0.10;
  public static final double simKd = 0.0;
  public static final double simKs = 0.01;
  public static final double simKv = 0.227;
  public static final double simKa = 0.0;
  public static final double simKg = 0.0;
}
