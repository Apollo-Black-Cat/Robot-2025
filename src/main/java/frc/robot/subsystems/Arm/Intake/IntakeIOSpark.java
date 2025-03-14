// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  private final SparkMax ScThRollers =
      new SparkMax(Constants.Intake.smallRollerId, MotorType.kBrushed);
  private WPI_VictorSPX FrRoller = new WPI_VictorSPX(Constants.Intake.algaeRollersId);

  public IntakeIOSpark() {
    FrRoller.setInverted(Constants.Intake.algaeRollerInverted);
    FrRoller.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void runIntake(boolean isOn) {
    if (isOn) {
      FrRoller.setVoltage(Constants.Intake.speed * Constants.Intake.maxVoltage);
      ScThRollers.setVoltage(Constants.Intake.speed * Constants.Intake.maxVoltage);
    } else {
      FrRoller.setVoltage(-Constants.Intake.speed * Constants.Intake.maxVoltage);
      ScThRollers.setVoltage(-Constants.Intake.speed * Constants.Intake.maxVoltage);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void stopMotors() {
    ScThRollers.stopMotor();
    FrRoller.stopMotor();
  }
}
