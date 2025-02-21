// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.Elevador;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.realKs;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.realKv;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.simKs;
import static frc.robot.subsystems.Arm.ArmConstants.Angulador.simKv;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Elevador extends SubsystemBase {
  /** Creates a new Elevador. */
  ElevadorIO elevadorIO;

  ElevadorIOInputsAutoLogged inputs = new ElevadorIOInputsAutoLogged();
  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;
  private final SysIdRoutine sysId;

  public Elevador(ElevadorIO elevadorIO) {
    this.elevadorIO = elevadorIO;
    // configure SysID
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevadorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run the arm in a open loop by voltage* */
  public void runOpenLoop(double speed) {
    elevadorIO.setSpeed(speed);
  }

  public void runCloseLoop(double position) {
    elevadorIO.setPosition(position);
  }

  public void stop() {
    elevadorIO.stopMotor();
  }

  /** Return a command of a Quasistatic routine */
  public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Return a command of a Dynamic routine */
  public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
