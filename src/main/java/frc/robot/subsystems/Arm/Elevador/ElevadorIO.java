// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Elevador;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevadorIO {
  @AutoLog
  public static class ElevadorIOInputs {

    public double velocityMetersPerSec = 0.0;
    public double height = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }
  /** Updates the inputs* */
  public default void updateInputs(ElevadorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeight(double height) {}

  public default void stopMotor() {}

  public default double getHeight() {
    return 0;
  }
}
