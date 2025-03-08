// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Intake;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkMax smallRoller =
      new SparkMax(Constants.Intake.smallRollerId, MotorType.kBrushless);
  private WPI_VictorSPX algaeRollersMotor = new WPI_VictorSPX(Constants.Intake.algaeRollersId);

  private final RelativeEncoder smallRollerEncoder = smallRoller.getEncoder();

  public IntakeIOSpark() {
    var config = new SparkMaxConfig();
    config.smartCurrentLimit(Constants.Intake.CurrentLimit).voltageCompensation(12.0);
    config
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(2 * Math.PI / Constants.Intake.IntakeGearing)
        .velocityConversionFactor(2 * Math.PI / 60 / Constants.Intake.IntakeGearing);
    config.inverted(Constants.Intake.smallRollerInverted);
    tryUntilOk(
        smallRoller,
        5,
        () ->
            smallRoller.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void runCoralIntake(boolean isOn) {
    if (isOn) {
      algaeRollersMotor.setVoltage(Constants.Intake.speed * Constants.Intake.maxVoltage);
      smallRoller.setVoltage(Constants.Intake.speed * Constants.Intake.maxVoltage);
    } else {
      algaeRollersMotor.setVoltage(-Constants.Intake.speed * Constants.Intake.maxVoltage);
      smallRoller.setVoltage(-Constants.Intake.speed * Constants.Intake.maxVoltage);
    }
  }

  @Override
  public void runAlgaeIntake(boolean isOn) {
    if (isOn) {
      algaeRollersMotor.setVoltage(Constants.Intake.speed * Constants.Intake.maxVoltage);
    } else {
      algaeRollersMotor.setVoltage(-Constants.Intake.speed * Constants.Intake.maxVoltage);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // get the velocity of the motors
    ifOk(smallRoller, smallRollerEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    // get the angle of the encoder
    ifOk(smallRoller, smallRollerEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(
        smallRoller,
        new DoubleSupplier[] {smallRoller::getAppliedOutput, smallRoller::getBusVoltage},
        (value) -> inputs.leftAppliedVolts = value[0] * value[1]);
    ifOk(
        smallRoller,
        new DoubleSupplier[] {smallRoller::getOutputCurrent, smallRoller::getOutputCurrent},
        (value) -> inputs.leftCurrentAmps = value);
  }

  @Override
  public void stopMotors() {
    smallRoller.stopMotor();
  }
}
