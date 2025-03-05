// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Angulador;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class AnguladorIOSim implements AnguladorIO {
  /** Creates a new AnguladorIOSim. */
  private final DCMotor motor = DCMotor.getNEO(2);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private final PIDController armPID =
      new PIDController(Constants.Angulador.simKp, 0.0, Constants.Angulador.simKd);
  private final Encoder encoder = new Encoder(0, 1);
  private final SparkMax leftMotor =
      new SparkMax(Constants.Angulador.leftMotorId, SparkMax.MotorType.kBrushless);
  private final SparkMax rightMotor =
      new SparkMax(Constants.Angulador.rightMotorId, SparkMax.MotorType.kBrushless);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motor,
          Constants.Angulador.motorReduction,
          Constants.Angulador.MOI,
          Constants.Angulador.armLength,
          Constants.Angulador.armMinAngle,
          Constants.Angulador.armMaxAngle,
          true,
          0,
          Constants.Angulador.armEncoderDistancePerPulse,
          0.0);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  // create a mecanism2d to simulate the arm
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1, 0.4);
  private final LoggedMechanismRoot2d m_armPivot = mechanism.getRoot("ArmPivot", .30, .30);
  /* private final LoggedMechanismLigament2d m_armTower =
      m_armPivot.append(new LoggedMechanismLigament2d("ArmTower", 0.3412, -90)); */
  private final LoggedMechanismLigament2d m_arm =
      m_armPivot.append(
          new LoggedMechanismLigament2d(
              "Arm",
              1,
              Units.radiansToDegrees(sim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public AnguladorIOSim() {
    encoder.setDistancePerPulse(Constants.Angulador.armEncoderDistancePerPulse);
  }

  @Override
  public void updateInputs(AnguladorIOInputs inputs) {
    if (closedLoop) {
      leftMotor.set(leftAppliedVolts);
      rightMotor.set(rightAppliedVolts);
    }

    // Update simulation state
    sim.setInputVoltage(leftAppliedVolts);
    sim.update(0.02);
    encoderSim.setDistance(sim.getAngleRads());
    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightMotor.getOutputCurrent()};
    m_arm.setAngle(Units.radiansToDegrees(sim.getAngleRads()));
    Logger.recordOutput("Arm/Angulador", mechanism);
    SmartDashboard.putNumber("Angulador Angle", Units.radiansToDegrees(sim.getAngleRads()));
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
    leftAppliedVolts = voltage;
    rightAppliedVolts = voltage;
  }

  @Override
  public void setAngle(double angle) {
    closedLoop = true;
    armPID.setSetpoint(Units.degreesToRadians(angle));
    double output = armPID.calculate(sim.getAngleRads());
    leftMotor.setVoltage(output);
    rightMotor.setVoltage(output);
    leftAppliedVolts = output;
    rightAppliedVolts = output;
  }
}
