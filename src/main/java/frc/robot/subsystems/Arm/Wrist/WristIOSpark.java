// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Wrist;

import static frc.robot.Constants.Wrist.positionFactor;
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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class WristIOSpark implements WristIO {
  /** Creates a new AnguladorIOSpark. */
  private final SparkMax wristMotor =
      new SparkMax(Constants.Wrist.wristMotorId, MotorType.kBrushless);

  private final AbsoluteEncoder WristEncoder = wristMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController wristController = wristMotor.getClosedLoopController();

  public WristIOSpark() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(frc.robot.Constants.Wrist.currentLimit)
        .voltageCompensation(12.0)
        .absoluteEncoder
        .positionConversionFactor(Constants.Wrist.positionFactor)
        .velocityConversionFactor(Constants.Wrist.positionFactor / 60);
    config
        .closedLoop
        .pid(Constants.Wrist.realKp, 0.0, Constants.Wrist.realKd)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, positionFactor);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    // Apply config to motors
    config.inverted(Constants.Wrist.wristInverted);
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
    wristController.setReference(
        Units.degreesToRadians(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    // get the velocity of the motors
    ifOk(wristMotor, WristEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    // get the angle of the encoder
    ifOk(wristMotor, WristEncoder::getPosition, (value) -> inputs.positionRad = value);
    // get the angle of the encoder
    ifOk(
        wristMotor,
        WristEncoder::getPosition,
        (value) -> inputs.positionDeg = Units.radiansToDegrees(inputs.positionRad));
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
    SmartDashboard.putNumber("WristAngle", new Rotation2d(WristEncoder.getPosition()).getDegrees());
  }
}
