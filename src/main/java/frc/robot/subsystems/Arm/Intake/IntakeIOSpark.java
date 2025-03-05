// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Intake;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkMax roller =
      new SparkMax(Constants.Intake.IntakeMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = roller.getEncoder();

  public IntakeIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Intake.CurrentLimit)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / Constants.Intake.IntakeGearing) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor(
            (2.0 * Math.PI)
                / 60.0
                / Constants.Intake.IntakeGearing) // Rotor RPM -> Roller Radians/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        roller,
        5,
        () ->
            roller.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(roller, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(roller, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        roller,
        new DoubleSupplier[] {roller::getAppliedOutput, roller::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(roller, roller::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    roller.setVoltage(volts);
  }
}
