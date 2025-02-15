// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Angulador;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants;

public class AnguladorIOSim implements AnguladorIO {
  /** Creates a new AnguladorIOSim. */
  
  private final DCMotor motor = DCMotor.getNEO(2);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private final PIDController armPID = new PIDController(ArmConstants.Angulador.simKp, 0.0, ArmConstants.Angulador.simKd);
  private final Encoder encoder = new Encoder(0, 0);
  private final SparkMax leftMotor = new SparkMax(ArmConstants.Angulador.leftMotorId, SparkMax.MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(ArmConstants.Angulador.rightMotorId, SparkMax.MotorType.kBrushless);

  private final SingleJointedArmSim sim = new SingleJointedArmSim(motor, ArmConstants.Angulador.motorReduction,ArmConstants.Angulador.MOI,ArmConstants.Angulador.armLength,ArmConstants.Angulador.armMinAngle,ArmConstants.Angulador.armMaxAngle, true,0,ArmConstants.Angulador.armEncoderDistancePerPulse);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  //create a mecanism2d to simulate the arm
  private final Mechanism2d  mechanism = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = mechanism.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(sim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public AnguladorIOSim() {
    encoder.setDistancePerPulse(ArmConstants.Angulador.armEncoderDistancePerPulse);
    SmartDashboard.putData("MyMechanism", mechanism);
  }
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
  }
  public void setVoltage(double voltage) {
    closedLoop = false;
    leftMotor.set(voltage);
    rightMotor.set(voltage);
  }
  public void setVelocity(double radPerSec, double fFVolts) {
    closedLoop = true;
    leftAppliedVolts = fFVolts + armPID.calculate(sim.getVelocityRadPerSec(), radPerSec);
    rightAppliedVolts = fFVolts + armPID.calculate(sim.getVelocityRadPerSec(), radPerSec);
  }
}
