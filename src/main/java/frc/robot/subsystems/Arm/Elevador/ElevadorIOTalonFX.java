// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Elevador;

import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.isInverted;
import static frc.robot.subsystems.drive.DriveConstants.motorReduction;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevadorIOTalonFX implements ElevadorIO {
  /** Creates a new ElevatorIOTalonFX. */
  private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorId);

  private Orchestra mOrchestra = new Orchestra();

  private final StatusSignal<Angle> elevetorMotorPosition = elevatorMotor.getPosition();
  private final StatusSignal<AngularVelocity> elevatorMotorVelocity = elevatorMotor.getVelocity();
  private final StatusSignal<Voltage> elevatorMotorVoltage = elevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> elevatorMotorCurrent = elevatorMotor.getSupplyCurrent();

  private final TrapezoidProfile m_Profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              metersToRotation(ElevatorConstants.elevatorMotorMaxSpeed), metersToRotation(3.0)));
  TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // private final ElevatorFeedforward feedforward =
  //     new ElevatorFeedforward(
  //         ElevatorConstants.realKs,
  //         ElevatorConstants.realKg,
  //         ElevatorConstants.realKv,
  //         ElevatorConstants.realKa);

  public ElevadorIOTalonFX() {

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = ElevatorConstants.realKp;
    config.Slot0.kD = ElevatorConstants.realKd;
    config.Slot0.kS = ElevatorConstants.realKs;
    config.Slot0.kG = ElevatorConstants.realKg;

    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(config, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        elevetorMotorPosition,
        elevatorMotorVelocity,
        elevatorMotorVoltage,
        elevatorMotorCurrent);
    elevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevadorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevetorMotorPosition, elevatorMotorVelocity, elevatorMotorVoltage, elevatorMotorCurrent);
    inputs.height = rotationToMeters(elevetorMotorPosition.getValueAsDouble());
    inputs.velocityMetersPerSec = rotationToMeters(elevatorMotorVelocity.getValueAsDouble());
    inputs.appliedVolts = elevatorMotorVoltage.getValueAsDouble();
    inputs.currentAmps = new double[] {elevatorMotorCurrent.getValueAsDouble()};
  }

  @Override
  public void setHeight(double height) {
    m_goal = new TrapezoidProfile.State(metersToRotation(height), 0);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    m_setpoint = m_Profile.calculate(0.02, m_setpoint, m_goal);
    m_request.Position = m_setpoint.position;
    m_request.Velocity = m_setpoint.velocity;
    elevatorMotor.setControl(m_request);
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  @Override
  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  @Override
  public double getHeight() {
    return rotationToMeters(elevetorMotorPosition.getValueAsDouble());
  }

  public void periodic() {}

  public double metersToRotation(double meters) {
    return (meters / (2 * Math.PI * ElevatorConstants.drumRadius)) * motorReduction;
  }

  public double rotationToMeters(double rotations) {
    return (rotations * (2 * Math.PI * ElevatorConstants.drumRadius)) / motorReduction;
  }
}
