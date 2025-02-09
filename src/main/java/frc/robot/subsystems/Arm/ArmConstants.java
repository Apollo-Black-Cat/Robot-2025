// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class ArmConstants {
  public static class Angulador {
    //Motors ID
    public static final int leftMotorId = 5; 
    public static final int rightMotorId = 6;

    public static final boolean leftInverted = false;
    public static final boolean rightInverted = true;

    //Motor reductions
    public static final double motorReduction = 125.0;

    public static final double InitialAngle = 0.0;

    public static final int currentLimit = 60;

    // Velocity PID configuration
    public static final double realKp = 0.0;
    public static final double realKd = 0.0;
    public static final double realKs = 0.0;
    public static final double realKv = 0.1;

    public static final double simKp = 0.10;
    public static final double simKd = 0.0;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;


  }
}
