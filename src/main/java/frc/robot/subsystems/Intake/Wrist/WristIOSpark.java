// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Intake.IntakeConstants;
import java.util.function.DoubleSupplier;

public class WristIOSpark implements WristIO {
  private final SparkMax wristMotor =
      new SparkMax(IntakeConstants.WristConstants.wristMotorId, MotorType.kBrushless);

  private final SparkClosedLoopController wristMotorController =
      wristMotor.getClosedLoopController();
  private final AbsoluteEncoder absoluteEncoder = wristMotor.getAbsoluteEncoder();

  /** Creates a new WristIOSpark. */
  public WristIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.WristConstants.currentLimit)
        .voltageCompensation(12);
    config.closedLoop.pid(
        IntakeConstants.WristConstants.realKp, 0.0, IntakeConstants.WristConstants.realKd);
    config
        .encoder
        .velocityConversionFactor(
            2 * Math.PI / 60.0 / IntakeConstants.WristConstants.motorReduction)
        .positionConversionFactor(2 * Math.PI / IntakeConstants.WristConstants.motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config.inverted(IntakeConstants.WristConstants.wristMotorInverted);
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.set(voltage);
  }

  @Override
  public void setAngle(double angle) {
    wristMotorController.setReference(
        Units.degreesToRadians(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void stopMotor() {
    wristMotor.stopMotor();
  }

  public void updateInputs(WristIOInputs inputs) {
    // get the velocity of the motors
    ifOk(wristMotor, absoluteEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    // get the angle of the encoder
    ifOk(wristMotor, absoluteEncoder::getPosition, (value) -> inputs.positionRad = value);
    // get the applied voltage and amps of the motors
    ifOk(
        wristMotor,
        new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
        (value) -> inputs.leftAppliedVolts = value[0] * value[1]);
    ifOk(
        wristMotor,
        new DoubleSupplier[] {wristMotor::getOutputCurrent, wristMotor::getOutputCurrent},
        (value) -> inputs.leftCurrentAmps = value);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
