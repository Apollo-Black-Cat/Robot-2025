// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Intake;

/** Add your docs here. */
public interface IntakeIO {

  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
  }

  /*update inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runCoralIntake(boolean isOn) {}

  public default void runAlgaeIntake(boolean isOn) {}

  public default void stopMotors() {}
}
