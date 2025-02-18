// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Angulador;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.gearRadius;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.realKs;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.realKv;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.simKs;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.simKv;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Angulador extends SubsystemBase {
  /** Creates a new Angulador. */
  AnguladorIO anguladorIO;

  AnguladorIOInputsAutoLogged inputs = new AnguladorIOInputsAutoLogged();
  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;
  private final SysIdRoutine sysId;

  public Angulador(AnguladorIO anguladorIO) {
    this.anguladorIO = anguladorIO;
    // configure SysID
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    anguladorIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(-0.228, 0.0, 0.348, new Rotation3d(0.0, -inputs.positionRad, 0.0))
        });
  }

  /** Run the arm in a open loop by voltage* */
  public void runOpenLoop(double voltage) {
    anguladorIO.setVoltage(voltage);
  }

  public void runCloseLoop(double angle) {
    anguladorIO.setAngle(angle);
  }

  public void stop() {
    anguladorIO.setVoltage(0);
  }

  /** Return a command of a Quasistatic routine */
  public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Return a command of a Dynamic routine */
  public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @AutoLogOutput
  public double getAngle() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @AutoLogOutput
  public double getVelocity() {
    return inputs.velocityRadPerSec * gearRadius;
  }

  /** Returns the average velocity in radians/second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
