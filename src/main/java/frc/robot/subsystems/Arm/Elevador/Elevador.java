// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Elevador;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevador extends SubsystemBase {
  /** Creates a new Elevador. */
  ElevadorIO elevadorIO;

  ElevadorIOInputsAutoLogged inputs = new ElevadorIOInputsAutoLogged();
  // private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  // private final SysIdRoutine sysId;

  public Elevador(ElevadorIO elevadorIO) {
    this.elevadorIO = elevadorIO;
    // configure SysID
    /*sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
        */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevadorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run the arm in a open loop by voltage* */
  public void runOpenLoop(double voltage) {
    elevadorIO.setVoltage(voltage);
  }

  public void runCloseLoop(double height) {
    elevadorIO.setHeight(height);
  }

  public void stop() {
    elevadorIO.stopMotor();
  }

  public double getHeight() {
    return inputs.height;
  }
}
