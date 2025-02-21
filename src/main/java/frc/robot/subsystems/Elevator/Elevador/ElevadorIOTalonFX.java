// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.Elevador;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class ElevadorIOTalonFX implements ElevadorIO {
  /** Creates a new ElevatorIOTalonFX. */
  private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorId);

  public ElevadorIOTalonFX() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setSpeed(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {}

  @Override
  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  @Override
  public void updateInputs(ElevadorIOInputs inputs) {}

  public void periodic() {}
}
