// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmConstants {
  public static class Angulador {
    // Motors ID
    public static final int leftMotorId = 1;
    public static final int rightMotorId = 6;

    public static final boolean leftInverted = false;
    public static final boolean rightInverted = true;

    // Motor reductions
    public static final double motorReduction = 150.0;

    public static final double InitialAngle = 0.0;

    public static final int currentLimit = 60;

    public static final double maxVoltage = 10.0;

    // Velocity PID configuration
    public static final double realKp = 0.0;
    public static final double realKd = 0.0;
    public static final double realKs = 0.0;
    public static final double realKv = 0.1;

    public static final double simKp = 0.10;
    public static final double simKd = 0.0;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;

    // Arm configuration
    public static final double armLength = 0.5;
    public static final double armMass = 2.0;
    public static final double armMaxAngle = 180.0;
    public static final double armMinAngle = 0.0;
    public static final double armInitialAngle = 0.0;
    public static final double MOI = SingleJointedArmSim.estimateMOI(armLength, armMass);
    public static final double armEncoderDistancePerPulse = 2 * Math.PI / 4096.0;
    public static final double gearRadius = Units.inchesToMeters(0.6);
  }
}
