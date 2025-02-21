// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Wrist;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Intake.IntakeConstants.WristConstants.realKs;
import static frc.robot.subsystems.Intake.IntakeConstants.WristConstants.realKv;
import static frc.robot.subsystems.Intake.IntakeConstants.WristConstants.simKs;
import static frc.robot.subsystems.Intake.IntakeConstants.WristConstants.simKv;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  WristIO wristIO;

  private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;
  private final SysIdRoutine sysId;

  WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
    // configure SysID
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run the arm in a open loop by voltage* */
  public void runOpenLoop(double voltage) {
    wristIO.setVoltage(voltage);
  }

  public void runCloseLoop(double angle) {
    wristIO.setAngle(angle);
  }

  public void stop() {
    wristIO.stopMotor();
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
