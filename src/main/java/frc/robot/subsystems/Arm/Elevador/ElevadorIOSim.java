// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Elevador;

import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.carriageMass;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.distancePerPulse;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.drumRadius;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.maxHeight;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.minHeight;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.motorReduction;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKa;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKd;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKg;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKp;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKs;
import static frc.robot.subsystems.Arm.Elevador.ElevatorConstants.simKv;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevadorIOSim implements ElevadorIO {
  /** Creates a new ElevatorIOSim. */
  private double appliedVolts = 0.0;

  private boolean closedLoop = false;
  private final DCMotor elevatorGearbox = DCMotor.getKrakenX60(1);
  private final TalonFX m_motor = new TalonFX(ElevatorConstants.elevatorMotorId);
  // Standart classes for control the elevator
  private final ProfiledPIDController m_conController =
      new ProfiledPIDController(simKp, 0, simKd, new TrapezoidProfile.Constraints(1.5, 1.0));

  ElevatorFeedforward m_feedforward = new ElevatorFeedforward(simKs, simKg, simKv, simKa);
  private final Encoder m_encoder = new Encoder(2, 3);

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorGearbox, motorReduction, carriageMass, drumRadius, minHeight, maxHeight, true, 0);

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // create a mecanism2d to simulate the elevator
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(1, 1.100);
  private final LoggedMechanismRoot2d mech2Root = mech2d.getRoot("Elevator root", .30, .30);
  private final LoggedMechanismLigament2d elevatorMech2d =
      mech2Root.append(
          new LoggedMechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  public ElevadorIOSim() {
    m_encoder.setDistancePerPulse(distancePerPulse / motorReduction);
  }

  @Override
  public void updateInputs(ElevadorIOInputs inputs) {
    // Update the simulation of the elevator
    if (closedLoop) m_motor.set(appliedVolts);

    m_elevatorSim.setInput(appliedVolts);
    m_elevatorSim.update(0.020);
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    m_encoderSim.setRate(m_elevatorSim.getVelocityMetersPerSecond());
    // Update the mechanism2d
    elevatorMech2d.setAngle(SmartDashboard.getNumber("Angulador Angle", 90));
    elevatorMech2d.setLength(m_elevatorSim.getPositionMeters());
    // Update the inputs
    inputs.height = m_elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {m_elevatorSim.getCurrentDrawAmps()};

    Logger.recordOutput("Arm/Elevador", mech2d);
  }

  @Override
  public void setHeight(double height) {
    closedLoop = true;
    m_conController.setGoal(height);

    double pidOutput = m_conController.calculate(m_encoderSim.getDistance());
    double feedforward = m_feedforward.calculate(m_conController.getSetpoint().velocity);
    appliedVolts = pidOutput + feedforward;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    m_motor.set(volts);
  }

  @Override
  public void stopMotor() {
    appliedVolts = 0.0;
    m_motor.set(0.0);
  }
}
