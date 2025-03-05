// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {

  @AutoLog
  public static class WristIOInputs {

    public double velocityRadPerSec = 0.0;
    public double positionRad = 0.0;
    public double positionDeg = 0.0;

    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
  }
  /** Updates the inputs* */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setAngle(double angle) {}
}
