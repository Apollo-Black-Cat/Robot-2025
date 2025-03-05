// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Angulador;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class AnguladorIOSpark implements AnguladorIO {
  /** Creates a new AnguladorIOSpark. */
  private final SparkMax leftMotor =
      new SparkMax(Constants.Angulador.leftMotorId, MotorType.kBrushless);

  private final SparkMax rightMotor =
      new SparkMax(Constants.Angulador.rightMotorId, MotorType.kBrushless);
  private final RelativeEncoder ArmEncoder = leftMotor.getEncoder();
  private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

  public AnguladorIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(Constants.Angulador.currentLimit)
        .voltageCompensation(12.0);
    config.closedLoop.pid(Constants.Angulador.realKp, 0.0, Constants.Angulador.realKd);
    config
        .encoder
        .velocityConversionFactor(2 * Math.PI / 60.0 / Constants.Angulador.motorReduction)
        .positionConversionFactor(2 * Math.PI / Constants.Angulador.motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    // Apply config to motors
    config.inverted(Constants.Angulador.leftInverted);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.inverted(Constants.Angulador.rightInverted);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.set(voltage);
    rightMotor.set(voltage);
  }

  @Override
  public void setAngle(double angle) {
    leftController.setReference(
        Units.degreesToRadians(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
    rightController.setReference(
        Units.degreesToRadians(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void updateInputs(AnguladorIOInputs inputs) {
    // get the velocity of the motors
    ifOk(leftMotor, ArmEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    // get the angle of the encoder
    ifOk(leftMotor, ArmEncoder::getPosition, (value) -> inputs.positionRad = value);
    // get the angle of the encoder
    ifOk(
        leftMotor,
        ArmEncoder::getPosition,
        (value) -> inputs.positionDeg = Units.radiansToDegrees(value));
    // get the applied voltage and amps of the motors
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
        (value) -> inputs.leftAppliedVolts = value[0] * value[1]);
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getOutputCurrent, leftMotor::getOutputCurrent},
        (value) -> inputs.leftCurrentAmps = value);
    ifOk(
        rightMotor,
        new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
        (value) -> inputs.rightAppliedVolts = value[0] * value[1]);
    ifOk(
        rightMotor,
        new DoubleSupplier[] {rightMotor::getOutputCurrent, rightMotor::getOutputCurrent},
        (value) -> inputs.rightCurrentAmps = value);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", new Rotation2d(ArmEncoder.getPosition()).getDegrees());
  }
}
